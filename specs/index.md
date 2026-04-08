# Project: Adelino

RL training pipeline for a 5-DOF hobby-servo robot, from simulation through real-world deployment.

## Overview

- [Project Overview](project_overview.md) - Scope, robot specs, curriculum, and timeline

## Epics

| Epic | Status | Description |
|------|--------|-------------|
| [Epic 0: Environment Setup](epic0_environment_setup.md) | Complete | Isaac Sim, PyTorch, Isaac Lab installation |
| [Epic 1: Robot Definition](epic1_robot_definition.md) | In Progress | Blender export, MJCF/USD, physics properties |
| [Epic 2: Simulation Validation](epic2_simulation_validation.md) | Not Started | Physics validation, actuator tuning |
| [Epic 3: Training Pipeline](epic3_training_pipeline.md) | Not Started | RL environment, curriculum training |
| [Epic 4: Deployment Infrastructure](epic4_deployment_infrastructure.md) | Not Started | Arduino firmware, sim-to-real |

## Embodiment

The Adelino embodiment definition lives at `embodiment/` (within this repo) and also serves as the built-in example for the general [Embodiment Definition Guide](../../docs/embodiment-definition/index.md).
