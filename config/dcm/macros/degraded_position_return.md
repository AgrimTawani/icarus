---
schema: icarus.dcm.macro.v1
id: degraded-position-return
revision: "1"
tags: [recovery, estimator, return]
allowed_action_families: [safety]
allowed_actions: [hold, return_home, land]
preconditions:
  - Position or perception health is degraded but the control link is available.
  - Senior safety policy has not already selected a stronger response.
evidence_links:
  - config/dcm/macro_scenarios/degraded-position-return.json
model_policy:
  phases: [stabilize, recover]
  recovery_actions: [hold, return_home, land]
  evidence_requirements: [control_link_available, reviewed_home_solution]
---
# Degraded-position return

Do not continue navigation on unreliable position evidence. Prefer a bounded
hold while the senior estimator/failsafe policy evaluates recovery. Request
`return_home` only when the state says the route and home solution are valid;
otherwise retain the senior policy's hold or land directive. Never construct
coordinates and never describe uncertainty as authorization.
