"""Resident Qwen3.5-4B (mission runtime vision()) on the same 24 labelled frames as bench_vlm.py."""
import json, os, sys, time
sys.path.insert(0, '/home/agrim/Desktop/Icarus')
from pathlib import Path
from python.dcm.llama_runtime import LlamaCppRuntime
from python.dcm.runtime_config import RuntimeDescriptor
from python.perception.vlm import inspect_image
S = os.path.dirname(os.path.abspath(__file__)); FRAMES = os.environ.get('VISION_BENCH_FRAMES', '/home/agrim/Desktop/Icarus/output/laya-speed-20261106/vision-bench/frames')
labels = json.load(open(os.path.join(S, 'labels.json')))
QS = {'building': 'Is there a building or house visible in this image?', 'person': 'Is there a person visible in this image?', 'car': 'Is there a car visible in this image?'}
files = sorted(f for f in os.listdir(FRAMES) if f[:3] in labels and f.endswith(('.png', '.jpg')))
d = RuntimeDescriptor.from_manifest('/home/agrim/Desktop/Icarus/config/dcm/agent-runtime.json', role='primary', contract_version='direct-agent-v1', prompt_version='bench', vocabulary_version='bench')
rows = []
with LlamaCppRuntime(d) as runtime:
    vision = runtime.vision()
    for f in files:
        row = {'frame': f[:3]}
        for q, text in QS.items():
            t = time.perf_counter()
            r = inspect_image(Path(FRAMES) / f, text, None, server=vision, structured=True, result_type='boolean')
            row[q] = {'found': r.get('found'), 'answer': (r.get('answer') or '')[:60], 'ms': round((time.perf_counter() - t) * 1000), 'gold': labels[f[:3]][q]}
        rows.append(row); print(json.dumps(row), flush=True)
summary = {}
for q in QS:
    n = len(rows); ok = sum(1 for r in rows if r[q].get('found') == r[q]['gold']); none = sum(1 for r in rows if r[q].get('found') is None)
    fp = sum(1 for r in rows if r[q].get('found') is True and not r[q]['gold']); fn = sum(1 for r in rows if r[q].get('found') is False and r[q]['gold'])
    ms = sorted(r[q]['ms'] for r in rows)
    summary[q] = {'n': n, 'correct': ok, 'none': none, 'fp': fp, 'fn': fn, 'median_ms': ms[n // 2]}
print('SUMMARY', json.dumps(summary))
json.dump({'rows': rows, 'summary': summary}, open(os.path.join(S, 'resident_results.json'), 'w'), indent=1)
