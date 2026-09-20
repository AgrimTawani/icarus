"""ResourceSampler tests.

The peak matters more than any single reading, so these check that the
sampler actually tracks a maximum across samples rather than the last one,
and that it degrades to reporting None rather than raising when nvidia-smi
or the target process is unavailable.
"""

import unittest
from unittest import mock

from python.dcm.resource_sampler import ResourceSampler, _gpu_memory_used_mib


class GpuQueryTests(unittest.TestCase):
    def test_no_nvidia_smi_returns_none_rather_than_raising(self):
        with mock.patch("python.dcm.resource_sampler.shutil.which",
                        return_value=None):
            self.assertIsNone(_gpu_memory_used_mib())

    def test_a_failed_query_returns_none(self):
        failed = mock.Mock(returncode=1, stdout="")
        with mock.patch("python.dcm.resource_sampler.shutil.which",
                        return_value="/usr/bin/nvidia-smi"), \
             mock.patch("python.dcm.resource_sampler.subprocess.run",
                       return_value=failed):
            self.assertIsNone(_gpu_memory_used_mib())

    def test_multiple_gpu_lines_take_the_maximum(self):
        # A multi-GPU machine reports one line per device; the peak across
        # the whole card set is what matters for "does this model fit".
        result = mock.Mock(returncode=0, stdout="512\n2048\n")
        with mock.patch("python.dcm.resource_sampler.shutil.which",
                        return_value="/usr/bin/nvidia-smi"), \
             mock.patch("python.dcm.resource_sampler.subprocess.run",
                       return_value=result):
            self.assertEqual(_gpu_memory_used_mib(), 2048)


class PeakTrackingTests(unittest.TestCase):
    def test_the_peak_is_the_maximum_across_samples_not_the_last(self):
        sampler = ResourceSampler()
        readings = iter([1000, 4000, 2000])
        with mock.patch("python.dcm.resource_sampler._gpu_memory_used_mib",
                        side_effect=lambda: next(readings, 2000)):
            sampler._sample_once()
            sampler._sample_once()
            sampler._sample_once()
        self.assertEqual(sampler.peaks()["peak_vram_mib"], 4000)

    def test_no_gpu_and_no_process_reports_none_for_both(self):
        sampler = ResourceSampler()
        with mock.patch("python.dcm.resource_sampler._gpu_memory_used_mib",
                        return_value=None), \
             mock.patch.object(sampler, "_target_process", return_value=None):
            sampler._sample_once()
        peaks = sampler.peaks()
        self.assertIsNone(peaks["peak_vram_mib"])
        self.assertIsNone(peaks["peak_ram_mib"])

    def test_the_context_manager_samples_on_entry_and_exit(self):
        # A run shorter than the sampling interval must still produce a
        # reading rather than reporting nothing.
        with mock.patch("python.dcm.resource_sampler._gpu_memory_used_mib",
                        return_value=1500):
            with ResourceSampler(interval_s=60) as sampler:
                pass
            self.assertEqual(sampler.peaks()["peak_vram_mib"], 1500)

    def test_pid_provider_is_consulted_on_every_sample(self):
        # The runtime's subprocess may not exist yet when sampling starts and
        # may be replaced later by a deadline-triggered restart, so the
        # provider must be re-called rather than cached from construction.
        calls = []

        def provider():
            calls.append(1)

        sampler = ResourceSampler(pid_provider=provider)
        with mock.patch("python.dcm.resource_sampler._gpu_memory_used_mib",
                        return_value=None):
            sampler._sample_once()
            sampler._sample_once()
        self.assertEqual(len(calls), 2)


if __name__ == "__main__":
    unittest.main()
