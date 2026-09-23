---
schema: icarus.dcm.macro.v1
id: landing-site-survey
revision: "2"
tags: [landing, inspection]
allowed_action_families: [navigation, perception, safety]
allowed_actions: [hold, goto, orbit, assess_landing_zone, return_home, land]
preconditions:
  - Aircraft is airborne with fresh downward perception.
  - Candidate site is inside the reviewed mission boundary.
evidence_links:
  - config/dcm/macro_scenarios/landing-site-survey.json
---
# Landing-site survey

Approach only a mission-declared candidate, hold to stabilize, and call the
bounded landing-zone assessment. Assessment evidence is not permission to
land. If suitability is negative, uncertain, stale, or unavailable, do not
descend: hold, inspect another declared candidate, or return home according to
the senior policy. A `land` proposal remains subject to every normal guard.
