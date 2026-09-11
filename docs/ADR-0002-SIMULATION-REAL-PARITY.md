# ADR 0002 — Preserve Contracts Across Simulation and Real Flight

Status: accepted

Mission, safety, model, logging and evaluation code must depend on normalized
Icarus contracts rather than Gazebo topics or hardware SDK objects. Gazebo/SITL
and physical Pixhawk/sensors are adapters beneath those contracts. This costs
adapter work early but makes simulator tests meaningful and prevents a second
software stack from emerging for the aircraft.
