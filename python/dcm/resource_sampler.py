"""Sample peak GPU and system memory during an evaluation run.

Roadmap 11.5 asks for "peak VRAM and system RAM" as a scoring dimension. A
14B model and a 4B model can have identical accuracy and completely
different deployment costs, and that trade-off is invisible unless it is
measured rather than assumed from the quantization alone.

This samples on a background thread rather than instrumenting the model
adapter, because the peak matters more than any single reading and the
adapter should not have to know it is being measured.
"""

import shutil
import subprocess
import threading

try:
    import psutil
except ImportError:  # pragma: no cover - psutil is a project dependency
    psutil = None


def _gpu_memory_used_mib():
    """Query nvidia-smi directly rather than depending on a Python binding.

    Returns None off a machine with no NVIDIA GPU, or none visible, rather
    than raising: CPU-only evaluation is a real case, not a broken one.
    """
    if not shutil.which("nvidia-smi"):
        return None
    try:
        output = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if output.returncode != 0:
        return None
    values = [int(line.strip()) for line in output.stdout.splitlines()
              if line.strip().isdigit()]
    return max(values) if values else None


class ResourceSampler:
    """Tracks peak VRAM and RSS over its own lifetime, on a background thread.

    Use as a context manager around the section being measured:

        with ResourceSampler() as sampler:
            ...
        peaks = sampler.peaks()
    """

    def __init__(self, interval_s=1.0, pid_provider=None):
        """`pid_provider`, if given, is called on every sample to find the
        model runtime's own process (llama-server), not the caller's, since
        the model is what is being measured. It is a callable rather than a
        fixed pid because the runtime process may not exist yet when
        sampling starts (its subprocess starts lazily on first decision) and
        may be replaced later if the deadline forces a restart. Returning
        None from it, or passing nothing, falls back to the current process.
        """
        self.interval_s = interval_s
        self._pid_provider = pid_provider
        self._peak_vram_mib = None
        self._peak_ram_mib = None
        self._stop = threading.Event()
        self._thread = None

    def _target_process(self):
        if not psutil:
            return None
        pid = self._pid_provider() if self._pid_provider else None
        try:
            return psutil.Process(pid) if pid else psutil.Process()
        except psutil.NoSuchProcess:
            return None

    def _sample_once(self):
        vram = _gpu_memory_used_mib()
        if vram is not None:
            self._peak_vram_mib = max(vram, self._peak_vram_mib or 0)
        process = self._target_process()
        if process is not None:
            try:
                ram_mib = process.memory_info().rss / (1024 * 1024)
            except psutil.NoSuchProcess:
                return
            self._peak_ram_mib = max(ram_mib, self._peak_ram_mib or 0)

    def _run(self):
        while not self._stop.is_set():
            self._sample_once()
            self._stop.wait(self.interval_s)

    def __enter__(self):
        self._sample_once()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=self.interval_s * 2)
        self._sample_once()

    def peaks(self):
        return {
            "peak_vram_mib": (round(self._peak_vram_mib, 1)
                              if self._peak_vram_mib is not None else None),
            "peak_ram_mib": (round(self._peak_ram_mib, 1)
                             if self._peak_ram_mib is not None else None),
        }
