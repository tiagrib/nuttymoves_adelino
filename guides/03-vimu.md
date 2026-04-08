# VIMU (Vision-based Proprioception) Guide

Step-by-step instructions for collecting training data, training the VIMU model, exporting to ONNX, running inference, and deploying the full Adelino pipeline.

## 1. Camera Setup

- Use a USB webcam at 640x480 resolution.
- Ensure good, consistent lighting. Avoid strong backlighting or shadows that move.
- Position the camera so the robot fills roughly 30-60% of the frame. The full robot should be visible in all poses.
- Mount the camera on a tripod or fixed surface. It must not move between shots.

**IMPORTANT -- Current Limitation:** The camera must remain in the exact same position during both data collection and inference. The current VIMU model (ResNet-18 regression) is trained on images from a single viewpoint and will produce incorrect predictions if the camera is moved. Do not bump, reposition, or adjust the camera between data collection and deployment.

This limitation is addressed in the roadmap (see section 8):
- **Phase B** adds synthetic pre-training with domain-randomized viewpoints to improve robustness.
- **Phase A** migrates to keypoint detection + PnP geometric solving, which naturally handles arbitrary camera placement and enables auto-calibration.

## 2. Prerequisites

Before collecting data, ensure the controller is running (see `02-adelino-control.md`):

```
adelino-standalone run --port COM3 --ws-port 8765 --baud 115200
```

Activate the VIMU training Python environment:

```
pyenv-venv activate vimu-training
```

Ensure dependencies are installed:

```
pip install torch torchvision opencv-python numpy onnx onnxruntime websocket-client
```

## 3. Data Collection

With `adelino-standalone` running in another terminal, run the automated sweep:

```
python vimu/training/collect.py sweep --ws ws://localhost:8765 --camera 0 --num-joints 5 --num-poses 1500 --output-dir ./vimu_data
```

| Flag | Default | Description |
|------|---------|-------------|
| `--ws` | (required) | WebSocket URL of the running controller |
| `--camera` | `0` | Camera device index |
| `--num-joints` | `6` | Number of joints (use `5` for Adelino) |
| `--num-poses` | `1500` | Number of random poses to collect |
| `--settle` | `0.6` | Seconds to wait after each command for servos to settle |
| `--output-dir` | `./data` | Output directory for frames and labels |
| `--seed` | `42` | Random seed for reproducible pose generation |

**What happens:**
1. The script generates random joint configurations within safe limits.
2. For each pose, it sends a WebSocket command to the controller, waits for servos to settle, captures a camera frame, and records the commanded joint angles.
3. Output: `vimu_data/frames/000000.jpg, ...` and `vimu_data/labels.csv`.

**Duration:** ~15 minutes for 1500 poses at 0.6s settle time.

A preview window shows each captured frame. Press `q` to stop early.

### Tilted base data (optional)

To add base orientation labels (roll/pitch), use the interactive tilted mode:

```
python vimu/training/collect.py tilted --camera 0 --num-joints 5 --output-dir ./vimu_data
```

Manually tilt the robot to known angles, then press SPACE to capture and enter the angle values.

## 4. Training

```
python vimu/training/train.py --data-dir ./vimu_data --num-joints 5 --epochs 100
```

| Flag | Default | Description |
|------|---------|-------------|
| `--data-dir` | (required) | Path to collected data directory |
| `--num-joints` | `6` | Number of joints (use `5` for Adelino) |
| `--epochs` | `100` | Number of training epochs |
| `--batch-size` | `32` | Batch size |
| `--lr` | `1e-3` | Learning rate |
| `--val-split` | `0.15` | Fraction of data used for validation |
| `--output-dir` | `./checkpoints` | Where to save model checkpoints |
| `--joint-weight` | `1.0` | Loss weight for joint angle regression |
| `--base-weight` | `0.5` | Loss weight for base orientation regression |

**What happens:**
- Trains a ResNet-18 backbone with a regression head.
- Uses cosine annealing LR schedule and AdamW optimizer.
- Saves the best checkpoint (by validation joint MAE) to `checkpoints/best.pt`.
- Logs per-epoch metrics to `checkpoints/log.csv`.

**Expected results:**
- Validation joint MAE should reach below 5 degrees (~0.087 rad) for a good model.
- Training takes approximately 30 minutes on a GPU, longer on CPU.
- Monitor the output: `Joint MAE X.X degrees` is printed each epoch.

## 5. ONNX Export

```
python vimu/training/export_onnx.py --checkpoint ./checkpoints/best.pt --output ./vimu_adelino.onnx
```

This produces two files:
- `vimu_adelino.onnx` -- the ONNX model for the Rust inference engine.
- `vimu_adelino.json` -- metadata (input size, joint names, normalization constants) that the Rust inference engine reads alongside the model.

The export script validates the model by running a test inference through ONNX Runtime and checking the output shape.

## 6. Inference

Build and run the VIMU inference engine:

```
cd vimu/inference
cargo run --release --features camera -- --model ../../vimu_adelino.onnx --camera 0 --port 9001 --display
```

| Flag | Default | Description |
|------|---------|-------------|
| `--model` / `-m` | (required) | Path to the ONNX model file |
| `--meta` | (auto) | Path to metadata JSON (defaults to model path with `.json` extension) |
| `--camera` / `-c` | `0` | Camera device index |
| `--port` / `-p` | `9001` | WebSocket server port for broadcasting state |
| `--fps` | `60` | Target capture FPS |
| `--process-noise` | `10.0` | EKF process noise (higher = more responsive) |
| `--measurement-noise` | `0.01` | EKF measurement noise (lower = trust model more) |
| `--display` | off | Show OpenCV preview window with overlay |

**What happens:**
- Captures camera frames, runs the ONNX model, filters output through an Extended Kalman Filter (EKF).
- Broadcasts JSON state messages on WebSocket port 9001 at 100+ FPS.
- Each message contains per-dimension position, velocity, and acceleration estimates.
- The `--display` flag shows a live preview with colored bars and stats.

**Note:** The `--features camera` flag is required. Without it, the binary compiles without camera support and will exit with an error.

## 7. Full Deployment (3 Terminals)

The complete sim-to-real pipeline requires three processes running simultaneously:

### Terminal 1: Controller (Arduino bridge)

```
adelino-standalone run --port COM3 --ws-port 8765 --baud 115200 --calibration calibration.toml
```

### Terminal 2: VIMU Inference (vision to state)

```
cd vimu/inference
cargo run --release --features camera -- --model ../../vimu_adelino.onnx --camera 0 --port 9001 --display
```

### Terminal 3: Policy Runner (brain)

```
cd projects/adelino
pyenv-venv activate adelino-deploy
python deployment/policy_runner.py --policy path/to/policy.onnx
```

Full flags for `policy_runner.py`:

```
python deployment/policy_runner.py \
    --policy path/to/policy.onnx \
    --vimu-ws ws://localhost:9001 \
    --controller-ws ws://localhost:8765 \
    --rate 30
```

| Flag | Default | Description |
|------|---------|-------------|
| `--policy` | (required) | Path to exported RL policy (`.onnx` or `.pt`) |
| `--vimu-ws` | `ws://localhost:9001` | VIMU inference WebSocket URL |
| `--controller-ws` | `ws://localhost:8765` | Controller WebSocket URL |
| `--rate` | `30` | Control loop frequency in Hz |
| `--duration` | indefinite | Run for N seconds then stop |

**Data flow:**

```
Camera --> VIMU Inference (Rust, :9001)
               |
               | JSON state (joint positions, velocities, base orientation)
               v
         Policy Runner (Python)
               |
               | JSON command (joint target positions in radians)
               v
         adelino-standalone (Rust, :8765) --> Serial --> Arduino --> Servos
```

**Startup order:** Start Terminal 1 first (controller), then Terminal 2 (VIMU), then Terminal 3 (policy runner). The policy runner waits for VIMU data before entering the control loop.

Press Ctrl+C in Terminal 3 to stop. The policy runner returns the robot to neutral pose before disconnecting.

## 8. Roadmap

### Phase B: Synthetic Pre-training for Viewpoint Robustness

Pre-train the VIMU model on synthetic images rendered in Isaac Sim with domain randomization (viewpoint, lighting, background). This makes the regression model robust to moderate camera placement changes without requiring a full architectural change.

### Phase A: Keypoint Detection for Arbitrary Camera Placement

Migrate from direct regression to keypoint detection + PnP geometric solving. This is the principled long-term solution that:
- Works from any camera viewpoint without retraining.
- Enables auto-calibration: place the robot at neutral, detect keypoints, estimate camera pose, refine through commanded poses.
- Naturally handles camera movement during operation.
