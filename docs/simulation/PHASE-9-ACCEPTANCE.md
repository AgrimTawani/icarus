# Phase 9 Acceptance

Status: rigorous automated headless gate passed on 2026-09-17. Operator GUI inspection
is intentionally pending.

## Implemented boundary

- `RangeSource` and `ImageSource` interfaces separate vendor/Gazebo transport
  from perception algorithms.
- Every sample carries sensor ID, frame, capture/receive time and sequence.
- The Gazebo adapter converts the 360×16 LiDAR scan into the common range type.
- FLU/FRD transforms produce an expiring local-NED obstacle map.
- The Drone API exposes current and streaming `PerceptionSummary` RPCs.
- The local planner uses a bounded 1 m grid, 2.5 m obstacle inflation, eight-way
  A* search and collision-checked line-of-sight simplification.
- The safety supervisor commands BRAKE when perception is stale or an immediate
  frontal hazard enters the 2.5 m emergency envelope.

The downward terrain returns and calibrated vehicle envelope are removed before
frontal obstacle classification. Landing and RTL recovery remain permitted so
an avoidance intervention cannot prevent a safe recovery action.

## Automated results

The release test used the seeded `simulation-perception-stress` profile without
a Gazebo GUI. It combined walls, buildings, trees, physical sensor noise,
turbulent wind, 50% gusts and 60° direction swings. Autonomy consumed live
LiDAR; an independent scorer consumed Gazebo truth solely after planning to
measure the actual trajectory against every collision primitive.

| Check | Result |
| --- | --- |
| Live LiDAR freshness | Passed; final sequence 356 |
| Planner decision | `destination reached via safe detour` |
| Ground-truth collision result | Zero collisions |
| Minimum surface clearance including 0.35 m vehicle radius | 1.606 m (required 1.5 m) |
| Goal error | 0.540 m |
| Ground-truth trajectory samples | 699 |
| Actual/direct path length | 24.198 m / 21.584 m |
| Maximum tilt / ground speed | 12.03° / 3.01 m/s |
| Measured peak turbulent wind | 9.02 m/s across 609 samples |
| Landing/disarm | Passed |
| FLU/FRD normalized-frame parity | Passed |
| Live LiDAR dropout | Safety-aborted active action; BRAKE commanded; recovered |
| Goal inside inflated obstacle | Rejected |

Local machine-readable evidence:
`logs/phase9/rigorous_20260917T004023.json`.

## Reproduce headlessly

```bash
./scripts/start-sim --profile simulation-perception-stress
./scripts/start-autonomy
./scripts/test-phase9
```

Run each command in its own terminal. Stop the autonomy service and simulator
with Ctrl+C after the report is written.

The first manual visual inspection should add `--gui` to the same profile;
the operator should confirm clearance and path shape before changing the frozen
Phase 9 policy.
