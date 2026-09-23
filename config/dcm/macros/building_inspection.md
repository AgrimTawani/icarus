---
schema: icarus.dcm.macro.v1
id: building-inspection
revision: "1"
tags: [building, inspection, hover, perception]
allowed_action_families: [lifecycle, navigation, perception, safety]
allowed_actions: [arm, takeoff, goto, hold, detect, inspect_scene, return_home, land]
preconditions:
  - The target is a reviewed landmark or is localized by an approved perception method.
  - State, position, and obstacle evidence are fresh before navigation.
evidence_links:
  - config/dcm/macro_scenarios/building-inspection.json
---
# Building inspection

Establish flight readiness, visually confirm the reviewed target, navigate to
only a reviewed or independently localized observation waypoint, and hold
before collecting requested semantic evidence. Do not infer a person count
from prose; request the person detector. If the target, clearance, or evidence
cannot be confirmed, hold or return home. Finish safely on the ground.
