# NuttyMoves: RL Training Pipeline for Adelino Robot

**Train reinforcement learning policies for a 5-DOF hobby-servo robot, from simulation through real-world deployment**

---

## Project Overview

NuttyMoves is a complete pipeline for training RL policies using NVIDIA Isaac Lab for Adelino, a custom 5-DOF serial-chain robot. The robot uses PWM hobby servos controlled via Arduino and aims to learn dynamic behaviors like balancing on its base edges, controlled tipping, and potentially hopping/jumping.

### Why This Project?

- **Accessible Hardware**: Uses affordable hobby servos (~$50 total) instead of expensive torque-controlled actuators
- **Sim-to-Real Focus**: Designed from the start for real hardware deployment
- **Cross-Platform Ready**: MJCF robot definition works with both PhysX (Windows) and Newton (Ubuntu)
- **Educational**: Complete guide from CAD to deployment, suitable for learning RL robotics

---

## Adelino Robot Specifications

### Physical Characteristics

| Property | Value |
|----------|-------|
| **Total height** | 35 cm |
| **Degrees of freedom** | 5 (revolute joints) |
| **Control interface** | PWM hobby servos via Arduino |
| **Sensors (current)** | Servo position feedback only |
| **Sensors (planned)** | IMU (MPU6050 or BNO085) |
| **Total mass** | ~595g (base to head) |

### Kinematic Chain (floor to head)

| Joint | Distance to Previous | Servo Model | Torque | Speed (60°) | Mass |
|-------|---------------------|-------------|--------|-------------|------|
| **Base to J1** | 5.0 cm | HK15338 | 25 kg·cm | 0.21 sec | 175g |
| **J1 to J2** | 6.5 cm | HK15298B | 20 kg·cm | 0.16 sec | 66g |
| **J2 to J3** | 10.0 cm | HK15328A | 12.8 kg·cm | 0.20 sec | 58g |
| **J3 to J4** | 9.0 cm | HK15138 | 4.3 kg·cm | 0.17 sec | 38g |
| **J4 to J5 (head)** | 14.0 cm | HK15178 | 1.4 kg·cm | 0.09 sec | 10g |

**Note**: Servo torque in SI units: 1 kg·cm ≈ 0.098 N·m

### Joint Configuration

```
         [Head]  ← J5: Pitch/Nod
           |
        [Link4]  ← J4: Roll
           |
        [Link3]  ← J3: Pitch
           |
        [Link2]  ← J2: Pitch
           |
        [Link1]  ← J1: Yaw (Base rotation)
           |
        [Base]   ← Fixed to ground (with feet)
```

---

## Project Goals: Progressive Curriculum

Rather than jumping to complex behaviors immediately, we follow a progressive curriculum that builds skills incrementally:

### Task 1: Head Orientation Control (Foundation)
**Goal**: Control the 3 DOF orientation (roll, pitch, yaw) of the head (end effector)

**Observations**:
- Head orientation error (quaternion or euler angles)
- Joint positions and velocities
- Previous actions

**Actions**: 5 joint position targets

**Success Criteria**:
- Reach target orientation within 5° error
- Maintain orientation for 3+ seconds
- Smooth joint motions (low jerk)

**Why Start Here**:
- Tests basic kinematic chain control
- Validates MJCF and actuator models
- Simple reward function (orientation error)
- Foundation for all subsequent tasks

---

### Task 2: Head Pose Control (Position + Orientation)
**Goal**: Control 6 DOF pose of the head - 3 DOF orientation + 3 DOF position (offset from base)

**Observations**:
- Head pose error (position + orientation)
- Joint states
- Base orientation (from IMU)
- Previous actions

**Actions**: 5 joint position targets

**Success Criteria**:
- Reach target pose within 2cm position, 5° orientation
- Maintain pose for 5+ seconds
- Handle varying target positions in workspace

**New Challenges**:
- Coupled position and orientation control
- Workspace limits awareness
- Singularity avoidance

---

### Task 3: Pose + Body Posture Control
**Goal**: Maintain target head pose while also keeping a desired body configuration (secondary objective)

**Observations**:
- Head pose error (primary)
- Body shape error (joint configuration vs. target)
- Joint limits proximity
- Energy usage

**Actions**: 5 joint position targets

**Success Criteria**:
- Maintain head pose (primary goal)
- Keep body in approximate desired configuration
- Minimize energy consumption
- Smooth transitions between targets

**New Challenges**:
- Multi-objective optimization (pose + posture)
- Null-space exploitation (redundancy in 5-DOF)
- Comfort/natural posture learning

---

### Task 4: Edge Balancing (Dynamic Stability)
**Goal**: Balance robot on one edge of its base while maintaining head orientation

**Observations**:
- Base tilt and angular velocity (IMU)
- Contact forces/locations
- Head orientation
- Joint states

**Actions**: 5 joint position targets

**Success Criteria**:
- Balance on edge for 10+ seconds
- Recover from small perturbations
- Maintain head roughly upright

**New Challenges**:
- Dynamic stability (not just kinematic control)
- Contact reasoning
- Fast reaction to disturbances

---

### Task 5: Controlled Tipping and Locomotion
**Goal**: Learn rocking/tipping motions for locomotion via dragging/shuffling

**Observations**:
- Base position and velocity
- Multi-contact state
- IMU data
- Target direction

**Actions**: 5 joint position targets

**Success Criteria**:
- Move base in target direction
- Maintain control during tipping
- Avoid falling completely over
- Repeatable locomotion gait

**New Challenges**:
- Momentum-based movement
- Multi-contact dynamics
- Directed locomotion

---

### Task 6: Hopping/Jumping (Stretch Goal)
**Goal**: Use coordinated joint movements to hop or jump

**Observations**:
- Ground contact state
- Vertical velocity
- Joint torques
- IMU acceleration

**Actions**: 5 joint position targets

**Success Criteria**:
- Lift off ground briefly
- Land stably without falling
- Repeat hops reliably

**New Challenges**:
- Explosive movement generation
- Aerial phase control
- Impact absorption

---

## Curriculum Rationale

This progression follows established robotics learning principles:

1. **Start Simple**: Orientation control only (Task 1)
2. **Add Complexity Gradually**: Position (Task 2), posture (Task 3)
3. **Introduce Dynamics**: Balancing (Task 4)
4. **Goal-Directed Behavior**: Locomotion (Task 5)
5. **High-Risk Maneuvers**: Jumping (Task 6)

Each task builds on skills from previous tasks:
- Task 2 requires Task 1's orientation control
- Task 3 adds posture awareness to Task 2
- Task 4 uses pose control from Task 2-3 while adding balance
- Task 5 exploits tipping dynamics learned in Task 4
- Task 6 requires all previous skills plus explosive power

**Training Strategy**: Train each task independently, then consider:
- **Transfer learning**: Initialize Task N+1 with Task N's policy
- **Multi-task learning**: Train on multiple tasks simultaneously
- **Curriculum learning**: Automatically progress from easy to hard tasks

---

## Technology Stack

### Simulation & Training
- **Isaac Lab** - GPU-accelerated robotics simulation framework
- **PhysX** - Physics engine (Windows compatible, current choice)
- **Newton** - Optional upgrade (Ubuntu only, 10-100x faster for large-scale training)
- **RSL-RL** - PPO implementation from ETH Zurich
- **PyTorch** - Neural network policies

### Robot Definition
- **MJCF** - MuJoCo XML format (cross-compatible with PhysX and Newton)
- **USD** - Universal Scene Description (Isaac Sim native format)
- **OBJ** - Mesh format for visual and collision geometry

### Real-World Deployment
- **Arduino** - Servo control and IMU interfacing
- **Python** - Policy execution and serial communication
- **ONNX/TorchScript** - Exported policy formats

---

## Development Approach

### Phase 1: Simulation Setup
1. Define robot in MJCF with accurate kinematics
2. Export meshes from 3ds Max, simplify for collision
3. Convert MJCF to USD for Isaac Sim
4. Verify physics behavior matches expectations

### Phase 2: RL Training
1. Create Isaac Lab environment with observations (IMU, joints)
2. Design reward function for balancing behavior
3. Train policy with PPO (2048-4096 parallel environments)
4. Tune reward weights based on emergent behavior
5. Evaluate robustness with domain randomization

### Phase 3: Sim-to-Real Transfer
1. Add IMU to physical robot
2. Upload Arduino firmware for servo+IMU control
3. Export trained policy to TorchScript
4. Run policy on real robot at 30-50Hz
5. Iterate on simulation parameters to close sim-to-real gap

### Phase 4: Advanced Behaviors
1. Train edge-balancing variant
2. Explore locomotion through tipping
3. Investigate momentum-based jumping (ambitious)

---

## Key Design Decisions

### Why MJCF over URDF?
- **Cross-platform**: Works with both PhysX (Windows) and Newton (Ubuntu)
- **Better actuators**: Native support for position actuators matching hobby servos
- **Future-proof**: Upgrading to Newton requires zero robot definition changes
- **Constraints**: Native closed-loop constraint support (for future parallel mechanisms)

### Why IdealPD Actuators?
- **Matches hardware**: Hobby servos have internal position control
- **Sim-to-real**: Explicit PD computation ensures predictable behavior
- **Simple**: No need for complex DCMotor models without speed-torque curves
- **Portable**: Works identically on PhysX and Newton

### Why PhysX First?
- **Windows development**: Easier development environment on Windows
- **Good enough**: PhysX handles 2048+ parallel envs adequately for this scale
- **Upgrade path**: Can switch to Newton later for humanoid-scale projects
- **Proven**: PhysX has extensive Isaac Lab support and examples

---

## Expected Outcomes

### Technical Outcomes
- Functional MJCF robot definition with accurate dynamics
- Trained policy achieving 10+ second balancing (simulation)
- Successful sim-to-real transfer with 5+ second balancing (hardware)
- Documented sim-to-real gap and mitigation strategies

### Learning Outcomes
- End-to-end RL robotics pipeline experience
- Sim-to-real transfer techniques (domain randomization, observation noise)
- Actuator modeling and servo control
- MJCF format and Isaac Lab framework mastery

---

## Project Status

**Current Phase**: Documentation and planning
**Next Steps**: Robot definition and MJCF creation

See epic files in this folder for detailed implementation plans for each development phase.

---

## Documentation Structure

This project uses modular documentation:

- **This file** (`project_overview.md`) - High-level project description
- `workflow/setup_guide.md` - Environment setup and installation
- `epic*.md` - Implementation plans in this folder
- Guides (robot definition, training, deployment) created as needed

---

**Project Start Date**: January 17, 2026
**Estimated Duration**: 8-12 weeks (part-time development)
