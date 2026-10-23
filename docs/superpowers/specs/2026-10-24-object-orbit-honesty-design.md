# Object approach and orbit: measured geometry only

Date: 2026-10-24. Scope: `python/dcm/skills.py`, `python/dcm/target_motion.py`,
`python/dcm/agent_tools.py`, `config/dcm/prompts/system.md`, unit tests.

## Problem

Episodes `20261024T012220` and `20261024T014032` show `approach_target` and
`orbit` reporting `SUCCEEDED` with geometry that was never measured: the
forward-depth "centre" formula (`depth_fix`) saturates on frame-filling VLM
boxes and places the orbit centre 10–25 m ahead of the aircraft; the approach
declares "close enough" from the forward depth after LiDAR has lost the target
and the aircraft is 18 m past the house. Episode `20261023T223535` (7 m AGL)
shows the LiDAR approach working and the orbit then aborted by safety because
the orbit skill flew a side-view measurement arc inside the building's hull.

## Design

1. `orbit` on an object flies one circle around the registry's measured
   enclosure (`orbit_envelope`: hull radius + standoff + tracking reserve).
   When only one face has been measured (`orbit_usable` false), the unseen
   depth is assumed equal to the visible width: the centre is moved away from
   the viewer by half the visible width and the radius grows to enclose that
   square. The evidence states this assumption. No measurement arcs, no depth
   fallback. An object with no LiDAR hull fails with a message that says so.
2. `approach_target` uses LiDAR only. Losing the target returns `FAILED`
   (message names the last measured range and that the object is below the
   LiDAR's field of view). A re-measurement that jumps farther than the last
   one is an association fault, like the existing closer-jump check. The
   "re-acquire once and continue" loop is removed; the model may call the tool
   again, which re-localizes.
3. `depth_fix`, `depth_approach`, `depth_orbit`, `_depth_fallback`,
   `depth_ahead_m`, `_depth_reached` and their tests are deleted. Forward depth
   stays available as context; it no longer plans flight.
4. Prompt: "orbit / circle / fly around X" is `orbit` with a `visible_object`
   destination directly after the search. `approach_target` is for "go to /
   get close to X". No scene-specific values.

Altitude is unchanged: the aircraft orbits at its current altitude. The
Drone API collision slice is 1 m above to 5 cm below the vehicle; above the
roofline it cannot see buildings, which is reported, not worked around.

## Verification

Unit: `tests/unit/test_skills.py`, `test_target_motion.py`,
`test_object_tool_resolution.py`. Live, AirVLN env_6 with the default drone:
the 10 m house orbit, the same at 6 m, and vague phrasings (orbit, perimeter,
area count). Pass criterion: orbit centre lies inside the LiDAR hull of the
chosen object and the camera frames show it; not the model's `finish` text.
