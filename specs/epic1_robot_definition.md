# Epic 1: Robot Definition

**Type**: Feature
**Status**: In Progress
**Priority**: P0 (Blocking)
**Depends On**: Epic 0 (Environment Setup)

---

## Overview

Create the complete robot definition for Adelino, from Blender geometry through to a working simulation in Isaac Lab. This epic is split into two tasks:

1. **Task 1: Geometry & Visual Verification** - Get the robot geometry exported from Blender and loading correctly in Isaac Lab with proper axis alignment and joint pivots, using auto/default physics properties.
2. **Task 2: Physical Properties & Actuator Configuration** - Define accurate masses, inertia tensors, collision meshes, and actuator models matching the real servos.

---

## Task 1: Geometry Export & Visual Verification

**Status**: In Progress
**Goal**: Export robot geometry from Blender and verify it loads correctly in Isaac Lab with correct axes, joint pivots, and visual alignment. Uses auto/default physics properties.

### What has been accomplished

- [x] Blender project organized with one collection per link (`Base`, `Asset_1`..`Asset_5`)
- [x] Empty objects (`Segment_0`..`Segment_5`) placed as joint pivots in each collection
- [x] `export_config.json` created with kinematic chain, axis definitions, and collection mappings
- [x] Export script (`scripts/blender/export_robot_meshes.py`) runs successfully
- [x] All 6 visual meshes exported as OBJ (`base`, `link1`..`link4`, `head`)
- [x] `kinematics.json` auto-generated with joint offsets from Blender positions
- [x] `mjcf_config.json` created with placeholder inertial/actuator values
- [x] MJCF generated (`v1/mjcf/adelino.xml`) via `generate_mjcf.py`
- [x] USD conversion completed (auto-converted by Isaac Lab at runtime)
- [x] Robot loaded and visualized in Isaac Lab (`visualize_isaaclab.py`)

### Remaining items

- [x] **Verify/fix coordinate system alignment** - Some axes were manually flipped after export. The `coordinate_system` fields in `export_config.json` (`forward`, `up`) map directly to the Blender OBJ exporter's axis settings. Adjust these values to eliminate manual post-export axis flipping. Valid values: `X`, `-X`, `Y`, `-Y`, `Z`, `-Z`.
- [x] **Verify joint pivot positions** - Confirm each joint rotates around the correct point by testing joint articulation in Isaac Lab viewer
- [x] **Verify joint rotation directions** - Confirm each joint axis (yaw/pitch/roll) matches the physical robot
- [x] **Verify scale** - The `kinematics.json` shows joint origins with Z values up to ~10.8 units, but the physical robot is 35cm tall. Confirm units are consistent and the model scale is correct (Blender should be set to meters)
- [ ] **Document the `coordinate_system` config** - Add explanation of `forward`/`up` fields and their valid values to the export docs

### Acceptance Criteria

- Robot loads in Isaac Lab without manual axis corrections
- All joints rotate around the correct pivot point
- Joint rotation directions match the physical robot
- Visual mesh alignment is correct at all joint configurations
- Scale is consistent with the physical robot dimensions

### Current File Inventory

```
embodiment/
├── Adelino.blend                  # Blender source (collections + pivot empties)
├── export_config.json             # Blender export configuration
├── mjcf_config.json               # MJCF generation configuration (placeholder values)
└── v1/
    ├── kinematics.json            # Auto-generated joint offsets
    ├── meshes/visual/             # 6 OBJ + MTL files
    │   ├── base.obj / .mtl
    │   ├── link1.obj / .mtl
    │   ├── link2.obj / .mtl
    │   ├── link3.obj / .mtl
    │   ├── link4.obj / .mtl
    │   └── head.obj / .mtl
    └── mjcf/
        ├── adelino.xml            # Auto-generated MJCF
        └── adelino/               # Auto-generated USD (runtime conversion)
            ├── adelino.usd
            └── configuration/*.usd
```

---

## Task 2: Physical Properties & Actuator Configuration

**Status**: Not Started
**Goal**: Replace placeholder physics values with accurate measurements. Create collision meshes. Configure actuator models to match real servo behavior.
**Depends On**: Task 1 (geometry verified and correct)

### Subtasks

#### 2a: Measure and define link masses
- [ ] Measure or estimate mass of each link on the physical robot
- [ ] Update `mjcf_config.json` with accurate masses
- [ ] Validate total mass matches physical robot (~595g)

#### 2b: Calculate inertial properties
- [ ] Calculate center of mass for each link (from Blender geometry or measurement)
- [ ] Compute inertia tensors (Blender script, CAD approximation, or geometric primitives)
- [ ] Explore exporting inertial properties directly from Blender (script to compute volume, CoM, and inertia from mesh geometry + density)
- [ ] Update `mjcf_config.json` with computed values

#### 2c: Create collision meshes (optional)

By default, `generate_mjcf.py` reuses the visual meshes for collision (`use_visual_for_collision: true`). Separate collision meshes are only needed for performance optimization (2048+ envs), concave geometry issues, or self-collision problems.

- [ ] Evaluate whether visual-as-collision is sufficient for training
- [ ] If needed: simplify visual meshes (target < 500 triangles each)
- [ ] If needed: use convex decomposition for concave shapes
- [ ] If needed: export to `v1/meshes/collision/` and set `include_collision: true` in `mjcf_config.json`

#### 2d: Configure actuators
- [ ] Set per-joint torque limits matching servo specs
- [ ] Tune kp (stiffness) and kv (damping) for realistic servo behavior
- [ ] Add per-joint actuator overrides in `mjcf_config.json`

**Servo Torque Limits** (from datasheets):
| Joint | Servo | Torque (kg·cm) | Torque (N·m) |
|-------|-------|----------------|--------------|
| J1 | HK15338 | 25 | 2.45 |
| J2 | HK15298B | 20 | 1.96 |
| J3 | HK15328A | 12.8 | 1.25 |
| J4 | HK15138 | 4.3 | 0.42 |
| J5 | HK15178 | 1.4 | 0.14 |

#### 2e: Validate physics behavior
- [ ] Robot stands stable under gravity with fixed base
- [ ] Gravity response looks realistic (no jitter, no explosion)
- [ ] Joint articulation feels physically plausible
- [ ] Collision meshes prevent interpenetration

### Acceptance Criteria

- All masses, inertias, and CoM positions based on measurement (not placeholders)
- Collision meshes exist and are < 500 triangles each
- Per-joint actuator limits match servo datasheets
- Robot is physically stable in simulation
- Total mass within 5% of physical robot
- Simulation runs at target speed (2048+ envs)

---

## Completion Criteria (Full Epic)

- [ ] **Task 1**: Geometry loads correctly with proper axes, pivots, and scale
- [ ] **Task 2**: Physics properties are measured and accurate
- [ ] MJCF validates and loads correctly
- [ ] USD conversion successful
- [ ] Kinematics match physical robot
- [ ] Ready for Epic 2 (Simulation Validation)

---

## References

- `docs/embodiment-definition/` - Workflow guide (step-by-step procedures)
- `references/torque_servo_actuators_guide.md` - Actuator modeling reference
- Physical robot measurements and CAD
