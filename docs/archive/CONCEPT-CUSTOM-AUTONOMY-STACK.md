# Archived Concept: Custom Drone Autonomy Stack

> Historical design input retained for traceability. For the canonical current
> architecture, read [`../architecture/SOFTWARE-ARCHITECTURE.md`](../architecture/SOFTWARE-ARCHITECTURE.md).

## Vision

Develop a fully onboard, AI-native autonomous drone platform with:

-   No cloud/API dependency
-   Complete onboard reasoning
-   Natural language control
-   Modular hardware and software architecture

------------------------------------------------------------------------

## Hardware Stack

### Custom AI Flight Computer PCB

A custom motherboard-like PCB integrating:

-   **NVIDIA Jetson Thor**
    -   128 GB RAM
    -   High-performance AI inference
    -   Runs LLM, perception, planning and autonomy software
-   **Pixhawk 6X Flight Controller**
    -   Real-time flight control
    -   Sensor fusion
    -   Stabilization
    -   Motor control
    -   Safety failsafes

Both systems are directly integrated on a custom PCB for
ultra-low-latency communication.

------------------------------------------------------------------------

## Software Architecture

``` text
Human Voice
      ↓
Speech-to-Text
      ↓
DCM (Drone Cognitive Model)
      ↓
Drone API
      ↓
MAVLink
      ↓
Pixhawk / ArduPilot
      ↓
Motors & Drone
```

------------------------------------------------------------------------

## Drone Cognitive Model (DCM)

The DCM is the drone's AI brain.

### Components

### 1. Drone-Oriented LLM

Responsible for:

-   Understanding human commands
-   Mission planning
-   Decision making
-   Context awareness
-   Recovery reasoning

Examples:

-   "Take off"
-   "Search this area"
-   "Follow that vehicle"
-   "Return home"

------------------------------------------------------------------------

### 2. State Engine

Converts raw telemetry into structured state information.

Inputs:

-   GPS
-   IMU
-   Battery
-   LiDAR
-   Cameras
-   Ultrasonic sensors
-   ArduPilot telemetry

Output:

``` json
{
  "battery": 72,
  "gps": "healthy",
  "altitude": 35,
  "mission": "search"
}
```

------------------------------------------------------------------------

## Drone API

The Drone API acts as the interface between AI and aircraft.

Functions:

-   takeoff()
-   land()
-   rtl()
-   goto()
-   orbit()
-   get_battery()
-   get_gps()
-   get_parameters()

The LLM never directly controls hardware.

It uses Drone API functions.

------------------------------------------------------------------------

## MAVLink Layer

MAVLink is the communication layer between:

-   Drone API
-   ArduPilot
-   Pixhawk

Flow:

``` text
LLM
 ↓
Drone API
 ↓
MAVLink
 ↓
Pixhawk / ArduPilot
 ↓
Drone
```

------------------------------------------------------------------------

## Design Philosophy

-   AI performs reasoning
-   ArduPilot performs flight control
-   Drone API provides abstraction
-   Guardrails ensure safety
-   Everything runs fully onboard

------------------------------------------------------------------------

## Final Architecture

``` text
Sensors
   ↓
State Engine
   ↓
Drone Cognitive Model (LLM)
   ↓
Drone API
   ↓
MAVLink
   ↓
Pixhawk 6X
   ↓
Motors

Jetson Thor runs:
- DCM
- Perception
- Mission planning
- AI inference

Pixhawk runs:
- Stabilization
- Flight control
- Safety logic
```

------------------------------------------------------------------------

## Long-Term Goal

Create a general-purpose autonomous drone platform capable of:

-   Surveillance
-   Search and rescue
-   Cinematography
-   Inspection
-   Mapping
-   Consumer applications
-   Defense applications

All powered by a fully onboard Drone Cognitive Model.
