# Icarus Third-Party Source Revisions

External source trees are deliberately ignored and recreated beneath
`third_party/` by `scripts/install_dependencies.sh`. Icarus-owned changes must
not be made inside those trees.

## Known-Good Local Revisions

Recorded on 2026-09-12:

| Project | Official source | Commit |
| --- | --- | --- |
| ArduPilot | `https://github.com/ArduPilot/ardupilot.git` | `14c871f2732ccc5f5f558d003eb46fb2fe29d832` |
| ArduPilot Gazebo | `https://github.com/ArduPilot/ardupilot_gazebo.git` | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` |
| llama.cpp | `https://github.com/ggml-org/llama.cpp.git` | `1af554f8fc78ba029665a47b839484d9763e2a75` |

llama.cpp is fetched by `scripts/setup-model-runtime` rather than the
simulation bootstrap, because building it compiles CUDA kernels that only the
Phase 11 model runtime needs. It reads the same lock file, so the pinned
revision is authoritative either way. Pinning it matters for the same reason
the model checksum does: llama.cpp changes sampling defaults, chat-template
handling and GGUF parsing frequently, and an unpinned runtime would let those
changes silently alter Phase 12 comparisons.

These commits are enforced by
`config/dependencies/third_party.lock.json`. The bootstrap fetches and checks
out those immutable revisions instead of following upstream branches. Existing
dirty third-party checkouts are rejected rather than overwritten. To inspect
the active revisions:

```bash
./scripts/check-workspace --scope simulation
```

Review and comply with each upstream project's license. The supplied/custom
mesh provenance is documented separately in the `VEHICLE-*` documents and
`simulation/models/` metadata.

## Edge vision provenance

The experimental, simulation-only `edge-vision` profile uses
`ultralytics/yolo11` (`yolo11n`) for continuous 640 px detection/tracking.
Ultralytics is AGPL-3.0; this dependency is research-only until a commercial
licensing decision or a permissively licensed replacement is made. SmolVLM
(`HuggingFaceTB/SmolVLM-256M-Instruct`) is Apache-2.0 and is bounded
qualitative evidence only, with no flight authority.

The deterministic edge scenario also provisions version 1 of Mingfei's Gazebo
Fuel `actor` model to `~/models/edge/gazebo_fuel_cache/`. It is CC-BY-4.0 and
its per-file checksums are included in `~/models/edge/MANIFEST.json`. The
generated world uses only that local pinned copy; the simulator does not fetch
Fuel assets while a mission is running.
