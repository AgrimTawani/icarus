# ADR 0001: Deterministic Safety Owns Flight Authority

Status: accepted

Language models are probabilistic and may be slow, malformed or unavailable.
Therefore the DCM may emit only typed high-level actions. Guardrails validate
them, a deterministic mission executor performs them, an independent safety
supervisor may override them, and ArduPilot retains stabilization and native
failsafes. The consequence is less direct model control but auditable behavior
and safe degradation when the model or companion computer fails.
