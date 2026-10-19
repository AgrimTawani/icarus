# DRONE_API.md
Native tool schemas define calls and arguments; return a tool call, never prose code. Units: metres, m/s, seconds, degrees. Local coordinates are north/east; altitude_agl_m is above HOME, not terrain. face_heading is absolute compass yaw; yaw is a relative signed turn. Body-relative forward/right follows current heading.

Use user-supplied/measured coordinates or registered geometry. Missing named waypoints need clarification; camera pose is not object position. For visual destinations use localization/object-motion tools. Preserve explicit values; unspecified operating values use context.defaults.

The adapter supplies identity, lease, timestamps and origin, dispatches ActionService (proto/icarus/v1/drone_api.proto), and waits for a terminal result. ACCEPTED is not success. Short execute_sequence groups stop at failure and cannot cross observations/decisions. Routes/patterns delegate geometry and flight to ArduPilot.

Home/launch orbit references resolve from measured state. Select camera for visual intent, or none for heading-only flight. Nose-facing yaw cannot tilt a fixed camera; geometric feasibility and actual image evidence are separate. Object standoff is minimum distance from the observed hull, with reported tracking reserve; it is neither center radius nor global obstacle clearance. Unseen geometry stays unknown.

Scans own turn/path–settle–observe cycles. Select exit_policy: finish_at_match ends a detector search at its first matching station; pause_at_match allows resuming after a nested branch; each_observation yields for arbitrary decisions; complete_coverage runs an unconditional bounded detection scan. Match policies require stop_on_detection and optional minimum_count. A failed observation remains pending. A paused next station is optional, not an obligation to continue after the user's condition is satisfied. An unmatched sweep does not finish an open-ended search; sampled views do not establish complete visual coverage.

Older or large observation details stay in retained action results. Follow their action index/path with read_action_result when needed; lookup success is not flight success or fresh observation.
