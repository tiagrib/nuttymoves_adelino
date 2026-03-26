# Epic 2: Simulation Validation

**Type**: Feature
**Status**: Not Started
**Priority**: P1
**Depends On**: Epic 1 (Robot Definition)

---

## Overview

Validate that the simulated robot behaves realistically before training RL policies. Tune physics parameters, verify actuator response, and establish baseline metrics.

## Goals

- Verify robot dynamics match expectations
- Tune actuator PD gains for stable control
- Establish physics simulation parameters
- Create test scripts for ongoing validation
- Document expected vs actual behavior

---

## Tasks

### Task 2.1: Basic Physics Validation
**Status**: Not Started

- [ ] Drop robot from small height, verify stable landing
- [ ] Apply external forces, verify reasonable response
- [ ] Check joint limits are respected
- [ ] Verify no self-collision issues

**Output**: `training/scripts/test_physics.py`

**Acceptance Criteria**:
- Robot doesn't explode or fly away
- Contacts resolve correctly
- Joint limits enforced

---

### Task 2.2: Actuator Response Tuning
**Status**: Not Started

- [ ] Send step commands to each joint
- [ ] Measure settling time and overshoot
- [ ] Tune kp/kd for each joint
- [ ] Verify torque limits are effective

**Target Response**:
| Metric | Target |
|--------|--------|
| Settling time | < 0.5s for 45° step |
| Overshoot | < 10% |
| Steady-state error | < 2° |

**Acceptance Criteria**:
- All joints respond smoothly to commands
- No oscillation or instability
- Response approximately matches real servos

---

### Task 2.3: Gravity Compensation Test
**Status**: Not Started

- [ ] Start robot in neutral pose
- [ ] Measure joint torques needed to hold position
- [ ] Compare to expected values from statics
- [ ] Adjust inertial properties if needed

**Acceptance Criteria**:
- Robot holds pose under gravity
- Joint torques within servo capabilities
- No drift over time

---

### Task 2.4: Workspace Validation
**Status**: Not Started

- [ ] Command head to various positions in workspace
- [ ] Verify reachability matches expectations
- [ ] Identify singularities and joint limits
- [ ] Document achievable workspace

**Acceptance Criteria**:
- Head reaches expected positions
- Joint limits prevent impossible configurations
- Workspace matches physical robot

---

### Task 2.5: Performance Benchmarking
**Status**: Not Started

- [ ] Run simulation with increasing env counts
- [ ] Measure simulation speed (steps/second)
- [ ] Identify performance bottlenecks
- [ ] Establish target parallel env count

**Target Performance**:
| Env Count | Min Speed |
|-----------|-----------|
| 512 | Real-time |
| 2048 | > 0.5x real-time |
| 4096 | > 0.25x real-time |

**Acceptance Criteria**:
- 2048 envs runs stably
- Performance acceptable for training
- No memory issues

---

### Task 2.6: Create Validation Test Suite
**Status**: Not Started

- [ ] Compile all validation tests into suite
- [ ] Create pass/fail criteria for each test
- [ ] Document expected results
- [ ] Integrate with CI (optional)

**Output**: `tests/test_simulation.py`

**Acceptance Criteria**:
- All tests documented
- Tests can run automatically
- Clear pass/fail output

---

## Validation Checklist

### Physics
- [ ] Gravity direction correct
- [ ] Contact forces reasonable
- [ ] No interpenetration
- [ ] Stable at rest

### Actuators
- [ ] Position control works
- [ ] Torque limits enforced
- [ ] Response time acceptable
- [ ] No oscillation

### Kinematics
- [ ] Joint axes correct
- [ ] Joint limits enforced
- [ ] Forward kinematics verified
- [ ] Workspace reasonable

### Performance
- [ ] Runs at target env count
- [ ] No memory leaks
- [ ] Stable over long runs

---

## Completion Criteria

- [ ] All validation tasks complete
- [ ] Test suite created and passing
- [ ] Performance targets met
- [ ] Documentation updated with findings
- [ ] Ready to proceed to training

---

## Notes

- Document any discrepancies between expected and actual behavior
- Keep actuator gains and physics parameters in config files
- This epic may require iteration with Epic 1 (Robot Definition)
