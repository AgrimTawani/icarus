---
schema: icarus.dcm.macro.v1
id: perimeter-sweep
revision: "2"
tags: [inspection, survey]
allowed_action_families: [lifecycle, navigation, safety]
allowed_actions: [arm, takeoff, goto, orbit, hold, return_home, land]
preconditions:
  - Operator supplied a reviewed boundary and altitude.
  - State, position estimate, and local obstacle map are fresh.
evidence_links:
  - config/dcm/macro_scenarios/perimeter-sweep.json
---
# Perimeter sweep

Compose only typed actions. Establish flight readiness, visit the reviewed
boundary points in mission order, and use `hold` whenever the next leg lacks
fresh clearance evidence. Do not invent coordinates from this document.

On estimator degradation, geofence warning, repeated rejection, or an
unavailable local map, stop survey progress and request the senior recovery
route (`hold` or `return_home`). Finish on the ground with a guarded `land`.
This guidance never authorizes an action and never bypasses the executor.
