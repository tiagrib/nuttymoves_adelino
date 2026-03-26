# Epic 3: Training Pipeline

**Type**: Feature
**Status**: Not Started
**Priority**: P1
**Depends On**: Epic 2 (Simulation Validation)

---

## Overview

Build the RL training pipeline using Isaac Lab and RSL-RL. Implement the progressive curriculum from head orientation control through advanced behaviors.

## Goals

- Isaac Lab environment for Adelino robot
- Configurable observation and action spaces
- Reward functions for each curriculum task
- Training scripts with proper logging
- Evaluation and visualization tools

---

## Training Curriculum

The curriculum progresses through increasingly complex tasks:

| Task | Description | Prerequisite |
|------|-------------|--------------|
| 1 | Head Orientation Control | None |
| 2 | Head Pose Control (6-DOF) | Task 1 |
| 3 | Pose + Body Posture | Task 2 |
| 4 | Edge Balancing | Task 2-3 |
| 5 | Controlled Tipping/Locomotion | Task 4 |
| 6 | Hopping (Stretch Goal) | Task 5 |

---

## Tasks

### Task 3.1: Create Base Isaac Lab Environment
**Status**: Not Started

- [ ] Create environment class inheriting from Isaac Lab base
- [ ] Register robot USD asset
- [ ] Define simulation configuration (PhysX)
- [ ] Set up scene with ground plane

**Output**: `training/envs/adelino_env.py`

```python
class AdelinoEnv(DirectRLEnv):
    cfg: AdelinoEnvCfg

    def __init__(self, cfg, render_mode=None):
        super().__init__(cfg, render_mode)
        # Setup robot articulation
        # Setup scene
```

**Acceptance Criteria**:
- Environment initializes without errors
- Robot spawns correctly
- Basic step function works

---

### Task 3.2: Define Observation Space
**Status**: Not Started

Base observations (23-dimensional):
- Joint positions (5)
- Joint velocities (5)
- Base orientation (4, quaternion)
- Base angular velocity (3)
- Previous actions (5)
- Gravity vector in base frame (3) - derived from orientation

**Output**: Observation function in environment

**Acceptance Criteria**:
- Observations normalized to reasonable ranges
- Observation noise configurable (for domain randomization)
- Matches expected dimension

---

### Task 3.3: Define Action Space
**Status**: Not Started

Actions (5-dimensional):
- Joint position targets (normalized -1 to 1)
- Scaled to joint limits in environment

**Output**: Action handling in environment

**Acceptance Criteria**:
- Actions map correctly to joint targets
- Action rate limiting configurable
- Smooth transitions between actions

---

### Task 3.4: Implement Reward Functions
**Status**: Not Started

Create modular reward functions for each task:

**Task 1 - Orientation Control**:
```python
r_orientation = exp(-orientation_error^2 / sigma)
r_smoothness = -|action - prev_action|
r_total = w1 * r_orientation + w2 * r_smoothness
```

**Task 4 - Edge Balancing**:
```python
r_upright = exp(-tilt_error^2 / sigma)
r_height = exp(-|height - target|^2 / sigma)
r_alive = 1.0 if not fallen else 0.0
r_energy = -sum(|torques|)
r_total = w1*r_upright + w2*r_height + w3*r_alive + w4*r_energy
```

**Output**: `training/envs/rewards.py`

**Acceptance Criteria**:
- Rewards are differentiable-friendly (smooth)
- Weights configurable via config
- Reward components logged separately

---

### Task 3.5: Create Environment Configurations
**Status**: Not Started

- [ ] Base configuration class
- [ ] Task-specific configs (orientation, pose, balance, etc.)
- [ ] Domain randomization parameters
- [ ] Curriculum parameters

**Output**: `training/config/adelino_cfg.py`

**Acceptance Criteria**:
- All parameters configurable
- Configs can be overridden from command line
- Sensible defaults for each task

---

### Task 3.6: Implement Training Script
**Status**: Not Started

- [ ] RSL-RL PPO training setup
- [ ] Checkpoint saving
- [ ] Tensorboard logging
- [ ] Wandb integration (optional)

**Output**: `training/scripts/train.py`

```powershell
# Example usage
isaaclab.bat -p training/scripts/train.py \
    --task AdelinoOrientation \
    --num_envs 2048 \
    --max_iterations 5000
```

**Acceptance Criteria**:
- Training runs without errors
- Checkpoints saved periodically
- Metrics logged to tensorboard

---

### Task 3.7: Implement Evaluation Script
**Status**: Not Started

- [ ] Load trained checkpoint
- [ ] Run evaluation episodes
- [ ] Compute success metrics
- [ ] Optional visualization

**Output**: `training/scripts/evaluate.py`

**Acceptance Criteria**:
- Loads checkpoints correctly
- Reports meaningful metrics
- Visualization works (headless=False)

---

### Task 3.8: Implement Policy Export
**Status**: Not Started

- [ ] Export to TorchScript
- [ ] Export to ONNX (optional)
- [ ] Verify exported policy matches training

**Output**: `training/scripts/export_policy.py`

**Acceptance Criteria**:
- Exported policy produces same outputs
- File size reasonable for deployment
- Documentation for export format

---

### Task 3.9: Train Task 1 - Head Orientation
**Status**: Not Started

- [ ] Configure environment for orientation task
- [ ] Train policy for ~5000 iterations
- [ ] Evaluate success rate
- [ ] Document hyperparameters used

**Success Criteria**:
- Reach target orientation within 5° error
- Maintain orientation for 3+ seconds
- Success rate > 90%

---

### Task 3.10: Train Subsequent Tasks (2-6)
**Status**: Not Started

Implement and train each task in curriculum order:
- [ ] Task 2: Head Pose Control
- [ ] Task 3: Pose + Posture
- [ ] Task 4: Edge Balancing
- [ ] Task 5: Locomotion
- [ ] Task 6: Hopping (stretch goal)

**Note**: Each task may require multiple iterations of reward tuning.

---

## File Structure

```
training/
├── envs/
│   ├── __init__.py
│   ├── adelino_env.py       # Main environment class
│   └── rewards.py           # Reward function implementations
├── config/
│   ├── __init__.py
│   ├── adelino_cfg.py       # Environment configurations
│   └── ppo_cfg.py           # Training hyperparameters
└── scripts/
    ├── train.py             # Training entry point
    ├── evaluate.py          # Evaluation script
    └── export_policy.py     # Policy export for deployment
```

---

## Hyperparameters (Initial Guess)

| Parameter | Value |
|-----------|-------|
| Learning rate | 3e-4 |
| Batch size | 24576 |
| Mini-batches | 4 |
| Epochs per update | 5 |
| Discount (gamma) | 0.99 |
| GAE lambda | 0.95 |
| Clip range | 0.2 |
| Entropy coef | 0.01 |
| Value loss coef | 1.0 |

---

## Completion Criteria

- [ ] Base environment implemented and tested
- [ ] Task 1 successfully trained (> 90% success)
- [ ] At least Task 4 (balancing) attempted
- [ ] Training pipeline documented
- [ ] Policies exportable for deployment

---

## Notes

- Start with Task 1 and iterate before moving to harder tasks
- Log reward components separately for debugging
- Consider transfer learning between tasks
- Document all hyperparameter tuning experiments
