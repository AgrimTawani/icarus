# Icarus script map

Scripts are grouped by responsibility. Existing root commands are retained as
stable compatibility entry points; V2 does not delete or silently rename an
operator command used by prior acceptance evidence.

| Area | Canonical location | Stable entry points |
| --- | --- | --- |
| Gazebo simulation | `scripts/simulation/gazebo/` and existing builders in `scripts/simulation/` | `start-sim`, `sim`, `manual-control`, `view-camera` |
| AirSim simulation | `scripts/simulation/airsim/` | `airsim-doctor`, `fetch-airsim-blocks`, `start-airsim`, `airsim-smoke`, `stop-airsim` |
| Autonomous acceptance | `scripts/autonomy/` | `run-mission`, `test-phase8`, `test-phase9`, Phase 12 commands |
| DCM/evaluation | `python/dcm/` with root wrappers | `dcm-fly`, `observe-dcm`, `evaluate-dcm`, `compare-dcm` |
| Perception | `python/perception/` with root wrappers | `detect-image`, `detect-edge-image`, `assess-landing-zone` |
| Data/evidence | `python/dataset_tools/` with root wrappers | `replay-episode`, `export-candidates`, `phase12-campaign` |
| Setup/build | root `scripts/` and `scripts/lib/` | `bootstrap`, `check-workspace`, `generate-proto`, model setup commands |

Do not remove a legacy wrapper until repository search, documentation migration
and an explicit deprecation release establish that no automated workflow calls
it. Generated `__pycache__` folders and runtime logs are ignored artifacts,
not maintained scripts.
