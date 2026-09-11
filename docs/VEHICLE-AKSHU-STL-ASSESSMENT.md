# Vehicle: Akshu STL Assessment and Adaptation Boundaries

## Source and preservation

User-provided `/home/agrim/Downloads/akshu.STL`, SHA-256
`64f89254d2b68226b695534b8ea12a29d7ab912551ee680c2a2bf11897f152aa`.
The source is copied byte-for-byte into
`simulation/models/akshu_reference/meshes/akshu_original.stl`. Its creator,
license and component identities are unspecified; do not infer them from
appearance. No source geometry was repaired, deleted or deformed.

The separate `akshu_reference` Gazebo model is a static reference. Its body and
four propeller meshes retain all 266,716 input triangles. They are expressed
in metres, with the source camera-facing +Y direction mapped to body +X and
the motor-centre midpoint used as the horizontal origin. The provisional rotor
reference height is source Z=155.100006 mm. The transform and extracted motor
positions are recorded in `simulation/models/akshu_reference/extraction.json`.

## Measured geometry

STL carries no unit declaration. Millimetres are strongly supported by the
approximately 253 mm propeller diameter and 37 mm motor-housing diameter.
Dimensions below use that assumption; they are mesh measurements, not vendor
specifications or toleranced CAD measurements.

| Feature | Approximate dimensions |
| --- | --- |
| Whole exported assembly bounds | 572.7 × 529.8 × 278.4 mm |
| Motor-centre rectangle, source axes | 346.8 × 277.0 mm |
| Diagonal motor spacing | 443.8 mm |
| Propeller swept diameter from hub to furthest vertex | 253.2–253.3 mm |
| Individual motor-housing envelope | 37.2 × 37.2 × 33.7 mm |
| Electronics deck component | 56 × 140 × 4.6 mm |
| Upper plate component | 70.2 × 221.6 × 4 mm |
| Battery-shaped upper block | 49 × 155 × 36 mm |

The assembled STL includes arms, motor housings/shafts, three-blade propellers,
stacked boards/connectors, standoffs, landing legs, an antenna and a camera-like
front assembly. Identification of electronics is visual and provisional.

This is a rectangular/stretched X layout, not the existing 427 mm square-X
layout. The minimum adjacent motor spacing gives about 24 mm nominal propeller
tip clearance (23 mm with exactly 254 mm props), versus about 48 mm in the
existing reference. This geometric clearance is not a blade-flex or structural
validation.

## Mesh quality

Exact-coordinate welding gives 131,612 unique vertices, 227 connected shells,
68 boundary edges and 2,056 edges used by more than two triangles. There are no
nonfinite vertices or zero-area triangles at the inspector's tolerance.

Connected shells are not necessarily individual CAD parts: touching parts can
join and seams can split a part. These topology issues do not prevent visual
rendering, but volume-derived mass and a single solid collision mesh would not
be reliable. Preserve the detailed mesh for visuals and derive simple collision
shapes and independently justified inertia for the adapted flight model.

## Required changes for Icarus

1. Resolve motor geometry deliberately. Keeping this source layout requires new
   motor coordinates, inertia and renewed flight tests. Preserving the original
   427 mm square-X requirement requires redesigning arm geometry and mounting
   positions. Scaling the entire STL would also alter propellers, motors and
   fastener holes and is not an appropriate mechanical adaptation.
2. Rebuild the centre bay around the 140 × 100 mm compute envelope. The source
   56 mm electronics deck and roughly 70 mm upper plate are too narrow for that
   envelope without a new carrier/plate arrangement.
3. Replace the battery-shaped block with the selected pack's measured envelope.
   Our provisional 180 × 75 × 55 mm battery envelope is materially larger than
   the source block. Evaluate an underslung tray and recheck leg clearance/CG.
4. Give the Pixhawk, forward camera, downward rangefinder, GNSS and lidar explicit
   mounts and clear views. The source electronics must not be relabeled as the
   selected hardware without corresponding dimensions and mass allocations.
5. Recalculate mass, CG and inertia from the resulting layout before making this
   the flight model. The previous 4.343 kg model's tests do not validate this STL.

Only reference import, coordinate normalization and propeller separation are
complete at this checkpoint. Physical redesign and integration remain pending.

## Reproduction

```bash
/usr/bin/python3 scripts/simulation/inspect_stl.py /path/to/akshu.STL \
  --output logs/simulation/akshu_inspection/geometry.json
/usr/bin/python3 scripts/simulation/import_akshu_reference.py /path/to/akshu.STL
/usr/bin/python3 scripts/simulation/render_mark4.py \
  --model model://akshu_reference --vehicle-height 0.1551
```

The importer refuses a source with a different hash because its component IDs
are specific to this STL. The model is static and has no actuator forces,
assigned mass or claim of verified propeller handedness. Both the raw import
and the normalized reference were inspected using actual Gazebo camera renders.
