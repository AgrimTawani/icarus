"""The current Icarus VLM (Qwen2.5-VL-3B via llama-cli, load-infer-release, gpu_layers=99) on the same 24 frames."""
import json, os, sys, time
sys.path.insert(0, '/home/agrim/Desktop/Icarus')
from pathlib import Path
from python.perception.vlm import inspect_image
S = os.path.dirname(os.path.abspath(__file__)); FRAMES = os.environ.get('VISION_BENCH_FRAMES', '/home/agrim/Desktop/Icarus/output/laya-speed-20261106/vision-bench/frames')
labels = json.load(open(os.path.join(S, 'labels.json')))
manifest = Path.home() / 'models/edge/MANIFEST.json'
QS = {'building': 'Is there a building or house visible in this image?', 'person': 'Is there a person visible in this image?', 'car': 'Is there a car visible in this image?'}
files = sorted(f for f in os.listdir(FRAMES) if f[:3] in labels and f.endswith(('.png', '.jpg')))
rows = []
for f in files:
    row = {'frame': f[:3]}
    for q, text in QS.items():
        t = time.perf_counter()
        try:
            r = inspect_image(Path(FRAMES) / f, text, manifest, timeout_s=90, gpu_layers=99, structured=True, result_type='boolean')
            row[q] = {'found': r.get('found'), 'validity': r.get('validity'), 'answer': (r.get('answer') or '')[:60], 'ms': round((time.perf_counter() - t) * 1000), 'gold': labels[f[:3]][q]}
        except Exception as e:
            row[q] = {'found': None, 'error': str(e)[:120], 'ms': round((time.perf_counter() - t) * 1000), 'gold': labels[f[:3]][q]}
    rows.append(row); print(json.dumps(row), flush=True)
summary = {}
for q in QS:
    n = len(rows); ok = sum(1 for r in rows if r[q].get('found') == r[q]['gold']); none = sum(1 for r in rows if r[q].get('found') is None)
    fp = sum(1 for r in rows if r[q].get('found') is True and not r[q]['gold']); fn = sum(1 for r in rows if r[q].get('found') is False and r[q]['gold'])
    ms = sorted(r[q]['ms'] for r in rows)
    summary[q] = {'n': n, 'correct': ok, 'none': none, 'fp': fp, 'fn': fn, 'median_ms': ms[n // 2]}
print('SUMMARY', json.dumps(summary))
json.dump({'rows': rows, 'summary': summary}, open(os.path.join(S, 'vlm_results.json'), 'w'), indent=1)
