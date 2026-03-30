# nuttymoves-adelino — Project Custom Instructions

## Overview

This is the Adelino-specific project: Isaac Lab RL training + real-hardware deployment.

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

- Controller WebSocket: `../../metak-shared/api-contracts/controller-websocket.md`
- VIMU WebSocket: `../../metak-shared/api-contracts/vimu-websocket.md`
- Policy ONNX: `../../metak-shared/api-contracts/onnx-model-contract.md`

## Testing

- Training: verified by running Isaac Lab environments
- Deployment: manual integration test (3 processes: vimu, policy_runner, adelino-standalone)
