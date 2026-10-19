---
schema: icarus.dcm.macro.v1
id: building-inspection
revision: "1"
tags: [building, inspection, hover, perception]
allowed_action_families: [lifecycle, navigation, perception, safety]
allowed_actions: [arm, takeoff, goto, hold, detect, inspect_scene, return_home, land]
preconditions:
  - Local target coordinates are explicit operator inputs or come from an approved live 3-D localization capability.
  - State, position, and obstacle evidence are fresh before navigation.
evidence_links:
  - config/dcm/macro_scenarios/building-inspection.json
model_policy:
  phases: [launch, confirm_target, navigate, stabilize, collect_evidence, recover]
  recovery_actions: [hold, return_home, land]
  evidence_requirements: [operator_grounded_target, visual_inspection, requested_detection]
---
# Building inspection

Establish flight readiness, navigate only to operator-grounded or independently
localized live coordinates, stabilize, then inspect the current camera view and hold
before collecting requested semantic evidence. Do not infer a person count
from prose; request the person detector. If the target, clearance, or evidence
cannot be confirmed, hold or return home. Finish safely on the ground.
