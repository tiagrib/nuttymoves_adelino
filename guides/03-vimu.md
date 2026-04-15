# VIMU for Adelino

This guide covers Adelino-specific VIMU setup and deployment. For the full training pipeline, see the [VIMU Training Guide](../../../vimu/training/GUIDE.md).

## Prerequisites

1. Complete the [Adelino Control Guide](02-adelino-control.md) -- hardware, firmware, calibration
2. Have a working `calibration.toml`
3. Install VIMU Python dependencies (see [Training Guide prerequisites](../../../vimu/training/GUIDE.md#software))

## Adelino-Specific Notes

- Adelino has **5 joints** (pins 2-6)
- Use `--num-joints 5` where applicable
- Camera should see the full robot in all poses (640x480 minimum)

## Training Pipeline (summary)

VIMU v2 uses a segmentation-first pipeline. The full 6-phase process is documented in the [Training Guide](../../../vimu/training/GUIDE.md). Here's the Adelino-specific quick reference:

```bash
cd vimu/training/

# Phase 1: Film Adelino from various angles, annotate with SAM2
python annotate_seg.py --annotate-only
python annotate_seg.py --process-only

# Phase 2: Train segmentor
python train_segmentor.py --data seg_data/ --output vimu_seg.pt

# Phase 3: Collect pose data (with controller running in another terminal)
#   Terminal 1: adelino-standalone run --port COM3 --calibration calibration.toml
python collect_pose.py sweep \
    --calibration ../../projects/adelino/calibration.toml \
    --seg-model vimu_seg.pt --camera 0 --num-poses 500

# Phase 4-5: Train + export
python train.py --data ./pose_data --epochs 100
python export_onnx.py --checkpoint checkpoints/best.pt --output vimu_pose.onnx
python export_seg.py --model vimu_seg.pt --output vimu_seg.onnx
```

## Full Deployment (3 Terminals)

### Terminal 1: Controller (Arduino bridge)

```bash
cd projects/adelino
cargo run --release -p adelino-standalone -- run --port COM3 --calibration calibration.toml
```

### Terminal 2: VIMU Inference (vision to state)

```bash
cd vimu/inference
cargo run --release --features camera -- \
    --model ../training/vimu_pose.onnx \
    --seg-model ../training/vimu_seg.onnx \
    --camera 0 --port 9001 --display
```

### Terminal 3: Policy Runner (brain)

```bash
cd projects/adelino
pyenv-venv activate adelino-deploy
python deployment/policy_runner.py \
    --policy path/to/policy.onnx \
    --vimu-ws ws://localhost:9001 \
    --controller-ws ws://localhost:8765 \
    --rate 30
```

**Startup order:** Controller first, then VIMU, then Policy Runner.

**Data flow:**
```
Camera --> VIMU Inference (:9001) --> Policy Runner --> Controller (:8765) --> Arduino --> Servos
```

Press Ctrl+C in Terminal 3 to stop. The policy runner returns the robot to neutral before disconnecting.
