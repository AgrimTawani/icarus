# Gazebo Harmonic simulator

This is Icarus's current promotion simulator. It connects the project-owned
Gazebo Harmonic worlds and sensors to ArduPilot SITL through the existing
Gazebo bridge. It is the only simulator with current acceptance evidence.

Start a headless session with:

```bash
./scripts/start-sim --scenario wind_light
./scripts/start-autonomy
```

The canonical implementation remains under `scripts/simulation/` while the
public launcher lives here. `./scripts/start-sim`, `./scripts/sim`, camera
capture scripts, and existing Phase 8–12 commands remain supported aliases;
they are not removed in V2 because existing automation may call them.

See `docs/architecture/SIMULATION-CONNECTIONS.md` for the validated data flow
and `scripts/README.md` for the command map.
