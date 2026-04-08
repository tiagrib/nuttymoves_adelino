# nuttymoves-adelino — Project Custom Instructions

## Overview

This is the Adelino-specific project: standalone controller binary, Arduino firmware,
robot embodiment assets, Isaac Lab RL training, and real-hardware deployment.

## Structure

- `standalone/` — Rust binary: adelino-standalone (WebSocket-to-Arduino bridge)
- `firmware/` — Arduino C++ firmware (50Hz servo loop, serial protocol)
- `embodiment/` — Robot assets (Blender model, MJCF, USD, kinematics)
- `source/adelino_lab/` — Isaac Lab RL training (balance task, PPO)
- `deployment/` — Sim-to-real bridge (policy_runner, observation_builder)
- `guides/` — User documentation
- `specs/` — Architecture specs for future work

## Standalone Controller (standalone/)

- Rust binary, depends on `arduino-controller` library from nuttymoves via git
- Build: `cargo build --release -p adelino-standalone`
- Run: `cargo run --release -p adelino-standalone -- run --port COM3`

## Firmware (firmware/)

- Arduino C++ for Mega 2560
- Compile: `arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 firmware/`
- Flash: `adelino-standalone flash --port COM3`

## Training (source/adelino_lab/)

- Isaac Lab extension package, installed editable: `pip install -e source/adelino_lab`
- Environment: `Adelino-Balance-Flat-v0`
- Train: `python scripts/rsl_rl/train.py --task Adelino-Balance-Flat-v0`
- Play: `python scripts/rsl_rl/play.py --task Adelino-Balance-Flat-Play-v0`
- Embodiment path is currently hardcoded in `balance_env_cfg.py` (needs refactoring)

## Deployment (deployment/)

- `policy_runner.py` — bridges VIMU inference (WS :9001) → policy → controller (WS :8765)
- `observation_builder.py` — maps VIMU state to policy observation vector (26-dim)
- `config.py` — connection endpoints, joint config, observation layout
- Requires: `pip install -r deployment/requirements.txt`

## Integration Contracts

- Controller WebSocket: see nuttymoves `metak-shared/api-contracts/controller-websocket.md`
- VIMU WebSocket: see nuttymoves `metak-shared/api-contracts/vimu-websocket.md`
- Policy ONNX: see nuttymoves `metak-shared/api-contracts/onnx-model-contract.md`

## Testing

- Rust: `cargo test` (in this workspace)
- Python deployment: `pytest deployment/tests/`
- Training: verified by running Isaac Lab environments
- End-to-end: manual integration test (3 processes: vimu, policy_runner, adelino-standalone)
