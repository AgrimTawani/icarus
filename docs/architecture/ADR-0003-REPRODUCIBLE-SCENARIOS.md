# ADR 0003: Scenario Definitions Are Source; Generated Worlds Are Artifacts

Status: accepted

Versioned JSON scenarios and deterministic builders are committed. Generated
worlds, scenario-specific model copies and run logs are ignored. Each scenario
declares its seed, inputs and pass criteria. This keeps Git reviewable, avoids
stale generated files and allows CI or a new developer to recreate identical
assets from source.
