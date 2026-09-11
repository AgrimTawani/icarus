# Custom Akshu / Icarus Packaging Candidate V1

Superseded for simulation visuals by [the compact two-level chassis](../simulation/AKSHU-COMPACT.md).
Retained as design history; do not treat the tall stack as the current baseline.

This is an edited assembly STL and a static Gazebo packaging model, **not a
fabrication-ready chassis or a flight-validated replacement**. The original STL
and the existing Mark4 SITL model are unchanged.

## What was actually changed

Only twelve source mesh components are retained: four arms, four curved landing
legs, and four propeller visuals. The other 215 connected components, including
the original electronics, battery, camera, antenna, centre plates, motors and
shafts, are excluded from this candidate. Component numbers describe connected
mesh regions, not necessarily individual manufactured parts.

The centre is rebuilt with a 200 × 110 mm lower plate, a 150 × 110 mm elevated
compute shelf, hollow standoffs, a separate FC carrier, an ESC mount, battery tray
and restraint concepts. The source leg geometry is translated down by 35 mm and
joined with extension concepts. All four motors and shafts are rebuilt proxies;
they are not a relabel of the original motor meshes.

New payload geometry reserves space for a Thor module/custom carrier/cooling
assembly, Pixhawk 6X/baseboard, 6S 10 Ah battery, MID-360 lidar, forward stereo
camera, downward LW20/C and rear GNSS. The module carrier, cooling, battery,
camera enclosure and baseboard variant are not finalized: their meshes are
explicit **space allocations**, not exact vendor CAD.

## Outputs

- `simulation/models/akshu_icarus_v1/icarus_custom_assembly_mm.stl`: complete
  edited assembly in millimetres, body +X forward, +Y left, +Z up. Origin is at
  the source propeller reference height, not at the ground. STL has no unit
  metadata; select millimetres when importing into CAD.
- `simulation/models/akshu_icarus_v1/meshes/`: separate component meshes in
  metres for Gazebo, preserving independent visuals for subsequent work.
- `simulation/models/akshu_icarus_v1/model.sdf`: static visual model; no active
  sensors, rotor dynamics, collisions or flight-controller connection.
- `simulation/models/akshu_icarus_v1/packaging.json`: source hash, retained
  components, part descriptions, bounds and limited clearance checks.
- `simulation/models/akshu_icarus_v1/packaging_dimensions.png`: true orthographic
  top/front/side mesh projections with packaging dimensions.
- `logs/simulation/akshu_custom_v1_final/`: actual Gazebo camera renders from
  four viewpoints. These are not flight-test screenshots.

## Checks and limits

The builder verifies the original source SHA-256, rejects degenerate output
triangles, tests pairwise reserved payload boxes for overlap, and tests those
boxes against four 254 mm diameter rotor sweep cylinders spanning z=0..14 mm.
These checks pass. They do **not** check every structural triangle, flexible
propeller, connector, cable or real sensor viewing frustum.

The retained motor layout is approximately 443.8 mm diagonal. Nearest adjacent
254 mm propeller discs have approximately 23 mm edge separation. The shelf sits
20.5 mm above the conservative blade envelope. Ground clearance beneath the
battery tray is 45.6 mm on level ground; overall model height is 353.1 mm.
These are geometric dimensions, not margins validated for flight vibration or
landing deflection.

Mounting fasteners and plate holes, source-arm attachment compatibility,
landing-leg extension joints, wiring and connector access, retention under
load, GNSS visibility, sensor blind zones and cooling remain engineering work.
In particular, the lidar support above the compute cooler needs a thermal/load
path design; its presence here does not demonstrate sufficient cooling.

The combined STL contains intersecting assembly solids and inherited source
mesh defects; it is not boolean-unioned or intended to be printed as one part.
Do not derive mass or inertia from this STL's enclosed volume. Previous mass,
thrust-ratio and hover-test results apply to the previous model, not this layout.

## Reproduce

From the repository root, with existing system NumPy/Pillow/Gazebo bindings:

```bash
/usr/bin/python3 scripts/simulation/build_akshu_candidate.py
/usr/bin/python3 scripts/simulation/draw_akshu_packaging.py
gz sdf -k simulation/models/akshu_icarus_v1/model.sdf
/usr/bin/python3 scripts/simulation/render_mark4.py \
  --model model://akshu_icarus_v1 --vehicle-height 0.191
```

The renderer uses a unique Gazebo partition and stops its own process group.
No new dependencies were installed. Next gate is visual packaging review,
followed by actual component/interface CAD and revised mass/inertia before
integrating this candidate into the flight model.
