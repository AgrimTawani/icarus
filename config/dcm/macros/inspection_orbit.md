---
schema: icarus.dcm.macro.v1
id: inspection-orbit
revision: "1"
tags: [inspection, orbit, perception]
allowed_action_families: [lifecycle, navigation, perception, safety]
allowed_actions: [arm, takeoff, goto, orbit, hold, detect, inspect_scene, return_home, land]
preconditions:
  - Orbit center, radius, and altitude are explicit mission inputs.
  - Obstacle clearance and estimator state are fresh.
evidence_links:
  - config/dcm/macro_scenarios/inspection-orbit.json
---
# Inspection orbit

Navigate only to the declared staging point, execute the bounded orbit, then
request the minimum semantic perception needed by the mission. Do not repeat
an orbit or inspection that produced no new evidence. After two equivalent
failures, stop the inspection and request hold or return according to the
senior policy. End with the mission's guarded recovery and landing sequence.
