# Installation and Setup

This guide covers all prerequisites and environment setup for developing and deploying the Adelino robot.

## Prerequisites

### Required Software

| Tool | Version | Purpose |
|------|---------|---------|
| Python | 3.11+ | VIMU training, Isaac Lab, deployment scripts |
| Rust toolchain | stable | Controller binary, VIMU inference engine |
| arduino-cli | latest | Flashing Arduino firmware |
| OpenCV dev libs | 4.x | Camera capture (C++ libs for Rust `opencv` crate) |
| CUDA toolkit | 11.8+ | GPU-accelerated training (optional but recommended) |

### Hardware

- Arduino Mega 2560 with USB-B cable
- 5x hobby servos (HK15338, HK15298B, HK15328A, HK15138, HK15178)
- Separate 6V power supply for servos (do NOT power servos from the Arduino)
- USB webcam (640x480 minimum)

## pyenv-venv Setup

This project uses `pyenv-venv-win` to manage isolated Python virtual environments.

### Install a Python version

```
pyenv install 3.11.15
```

### List available Python versions and existing environments

```
pyenv-venv list python
pyenv-venv list envs
```

### Create and activate an environment

```
C:\Users\tiagr\.pyenv-win-venv\bin\pyenv-venv.exe install <env-name> 3.11.9
pyenv-venv activate <env-name>
```

## Python Environments

You need two separate Python environments: one for VIMU training (heavy ML dependencies) and one for Adelino deployment (lightweight).

### VIMU Training Environment

```
C:\Users\tiagr\.pyenv-win-venv\bin\pyenv-venv.exe install vimu-training 3.11.9
pyenv-venv activate vimu-training
```

Install dependencies:

```
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install opencv-python numpy onnx onnxruntime
```

This environment is used for:
- `vimu/training/collect.py` (data collection -- also needs `websocket-client`)
- `vimu/training/train.py` (model training)
- `vimu/training/export_onnx.py` (ONNX export)

If using the WebSocket controller backend for data collection, also install:

```
pip install websocket-client
```

### Adelino Deployment Environment

```
C:\Users\tiagr\.pyenv-win-venv\bin\pyenv-venv.exe install adelino-deploy 3.11.9
pyenv-venv activate adelino-deploy
```

Install dependencies:

```
pip install -r projects/adelino/deployment/requirements.txt
```

Which installs:
- `numpy>=1.24.0`
- `websocket-client>=1.6.0`
- `onnxruntime>=1.16.0`

This environment is used for:
- `projects/adelino/deployment/policy_runner.py`
- `projects/adelino/deployment/observation_builder.py`

## Rust Builds

Ensure the Rust toolchain is on your PATH:

```
export PATH="$USERPROFILE/.cargo/bin:$PATH"
```

### Build the Controller

```
cd controllers
cargo build --release
```

This produces the `adelino-standalone` binary in `target/release/`.

### Build the VIMU Inference Engine

```
cd vimu/inference
cargo build --release --features camera
```

The `--features camera` flag is required to enable the OpenCV camera capture and display functionality. Without it, the binary will refuse to run with an error message.

## Isaac Lab Setup

Isaac Lab is used for reinforcement learning training in simulation. Full installation instructions are in the [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/).

After installing Isaac Lab:

1. Install the Adelino Lab extension package:
   ```
   cd projects/adelino
   pip install -e source/adelino_lab
   ```

2. Set the `ADELINO_USD_PATH` environment variable or update the path in `source/adelino_lab/.../balance_env_cfg.py` to point to the Adelino USD asset. The embodiment path is currently hardcoded in `balance_env_cfg.py` and needs refactoring.

3. Verify the environment loads:
   ```
   python scripts/rsl_rl/train.py --task Adelino-Balance-Flat-v0 --num_envs 1 --headless
   ```

## Workspace Structure

```
nuttymoves/
├── controllers/                  Rust Arduino controller infrastructure
│   ├── arduino-controller/       Reusable library crate (serial, WebSocket, calibration)
│   ├── adelino-standalone/       Binary: WebSocket server bridging commands to Arduino
│   │   └── src/
│   │       ├── main.rs           CLI: run, flash, calibrate, test subcommands
│   │       └── adelino_config.rs Joint config (pins, PWM ranges, servo models)
│   └── firmware/
│       └── adelino/              Arduino C++ firmware (50Hz servo loop, serial protocol)
│           └── config.h          Pin assignments, PWM limits, timing, IMU toggle
│
├── vimu/                         Vision-based proprioception (standalone, no nuttymoves dependency)
│   ├── training/                 Python: data collection, ResNet-18 training, ONNX export
│   │   ├── collect.py            Sweep or tilted-base data collection
│   │   ├── train.py              Training loop with cosine LR, masked MSE loss
│   │   ├── export_onnx.py        Export to ONNX with metadata JSON
│   │   ├── model.py              ResNet-18 backbone + regression head
│   │   └── dataset.py            Dataset and augmentation transforms
│   └── inference/                Rust: real-time ONNX inference + EKF + WebSocket broadcast
│       └── src/main.rs           CLI: --model, --camera, --port, --display
│
├── projects/adelino/             Adelino-specific project (leaf node, depends on both above)
│   ├── source/adelino_lab/       Isaac Lab RL training (balance task, PPO)
│   ├── deployment/               Sim-to-real bridge
│   │   ├── policy_runner.py      Reads VIMU state, runs policy, sends commands (30Hz loop)
│   │   ├── observation_builder.py Maps VIMU state to 26-dim observation vector
│   │   ├── config.py             Connection endpoints, joint config, observation layout
│   │   └── requirements.txt      Deployment Python dependencies
│   ├── scripts/                  Training and playback scripts
│   └── guides/                   This documentation
│
├── embodiments/                  Robot definitions (MJCF, USD, kinematics)
│   └── adelino/                  Shared source of truth for Adelino geometry
│
├── scripts/                      Reusable Python utilities
│
└── metak-shared/                 Shared documentation (read-only for workers)
    ├── architecture.md           System boundaries and data flow diagrams
    ├── api-contracts/            WebSocket and serial protocol specs
    ├── coding-standards.md       Linting, commits, reviews
    └── glossary.md               Domain terminology
```

### Key dependency rules

- `controllers/` is reusable and has no dependency on `vimu/` or `projects/adelino/`.
- `vimu/` is standalone and has no dependency on `nuttymoves` or `projects/adelino/`.
- `projects/adelino/` is the leaf node that integrates both at runtime via WebSocket APIs.
- Cross-component communication happens exclusively through WebSocket JSON, shared embodiment files, and ONNX model files.
