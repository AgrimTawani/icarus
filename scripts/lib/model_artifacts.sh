#!/usr/bin/env bash
# Shared provenance helpers for local model setup scripts.

icarus_sha256() {
  sha256sum "$1" | cut -d' ' -f1
}

icarus_manifest_upsert() {
  local manifest="$1" artifact_json="$2" metadata_json
  if (( $# >= 3 )); then
    metadata_json="$3"
  else
    metadata_json='{}'
  fi
  python3 - "${manifest}" "${artifact_json}" "${metadata_json}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
artifact = json.loads(sys.argv[2])
metadata = json.loads(sys.argv[3])
if not isinstance(artifact, dict) or not artifact.get("role"):
    raise SystemExit("artifact must be an object with a role")
if not isinstance(metadata, dict):
    raise SystemExit("manifest metadata must be an object")
manifest = json.loads(path.read_text()) if path.exists() else {}
items = manifest.setdefault("artifacts", [])
manifest["artifacts"] = [item for item in items if item.get("role") != artifact["role"]]
manifest["artifacts"].append(artifact)
manifest.update(metadata)
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY
}
