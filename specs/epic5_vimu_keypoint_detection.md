# Epic 5: VIMU Keypoint Detection + Geometric Solving (Phase A)

**Type**: Model Architecture / Computer Vision
**Status**: Planned
**Priority**: P2
**Depends On**: Phase B (synthetic pre-training infrastructure)

---

## Overview

Migrate VIMU from direct angle regression to keypoint detection + geometric solving for principled viewpoint invariance and auto-calibration. The current `VimuModel` (in `vimu/training/model.py`) uses a ResNet-18 backbone with a fully-connected regression head that directly maps a 224x224 RGB image to `[joint_1, ..., joint_5, base_roll, base_pitch]`. This approach is inherently tied to the camera pose seen during training -- any change in camera placement requires recollecting data and retraining.

Phase A replaces the regression head with a heatmap decoder that detects 2D keypoint locations on the robot body. Given known 3D geometry (from `embodiment/v1/kinematics.json`), a PnP solver recovers camera pose and an inverse kinematics solver recovers joint angles. This decouples the vision model from camera placement entirely and enables a one-time auto-calibration workflow.

## Goals

- Detect 7 2D keypoints on the robot body from any camera angle
- Solve for camera extrinsics via PnP (Perspective-n-Point)
- Recover 5 joint angles via inverse kinematics (reprojection minimization)
- Enable auto-calibration from arbitrary camera placement without retraining
- Maintain real-time inference at 60+ FPS on GPU

---

## Architecture

Full pipeline overview:

```
Camera Frame (640x480 RGB)
    │
    ▼
Resize + ImageNet Normalize (224x224)
    │
    ▼
Keypoint CNN (ResNet-18 backbone + transposed-conv decoder)
    │
    ▼
7 Heatmaps (56x56 each)
    │
    ▼
Soft-Argmax → 7 x (u, v) pixel coordinates
    │
    ├─── [calibration mode] ──► PnP Solver ──► Camera Pose (R, t) ──► Save vimu_calibration.json
    │
    └─── [inference mode] ──► IK Solver (FK + reprojection minimization)
                                    │
                                    ▼
                              5 joint angles + base_roll + base_pitch
                                    │
                                    ▼
                              EKF (existing) → WebSocket broadcast
```

The output contract (VIMU WebSocket API, `metak-shared/api-contracts/vimu-websocket.md`) remains unchanged -- downstream consumers (policy_runner.py) still receive the same `dims` array with joint positions, velocities, and accelerations. The change is entirely internal to the VIMU pipeline.

---

## Keypoint Definition

Seven keypoints are defined at anatomically significant locations on the Adelino robot. Their 3D positions in the base frame at neutral pose (all joints = 0) are derived from the joint origins in `embodiment/v1/kinematics.json`:

| Index | Name | 3D Position (meters, base frame, neutral pose) | Description |
|-------|------|-------------------------------------------------|-------------|
| 0 | `base_center` | (0.000, 0.000, 0.000) | Base origin, ground contact |
| 1 | `joint_1_top` | (0.000, 0.000, 0.04264) | Top of base, J1 rotation axis |
| 2 | `joint_2` | (0.000, 0.02716, 0.11053) | Shoulder joint (J2 pitch axis) |
| 3 | `joint_3` | (0.000, 0.01694, 0.20873) | Elbow joint (J3 pitch axis) |
| 4 | `joint_4` | (-0.01565, -0.00135, 0.31205) | Wrist roll joint (J4 roll axis) |
| 5 | `joint_5` | (0.000, -0.00135, 0.42749) | Head yaw joint (J5 rotation axis) |
| 6 | `head_tip` | (0.000, -0.00135, 0.47749) | Head top (estimated ~50mm above J5) |

**Notes:**
- Positions are in the base coordinate frame (Z-up) at neutral pose (all joints at 0 radians).
- Keypoints 1-5 correspond directly to joint origins from `kinematics.json`.
- Keypoint 0 (base_center) and keypoint 6 (head_tip) are fixed reference points that aid PnP stability. Keypoint 6 offset (0.050 m above J5) should be measured from the physical robot and updated.
- During inference, keypoints 2-6 move as joints articulate; keypoints 0-1 are fixed in the base frame.

---

## Model Architecture

### Backbone

Reuse the ResNet-18 ImageNet-pretrained backbone, identical to the current `VimuModel`:

```python
backbone = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
encoder = nn.Sequential(*list(backbone.children())[:-2])  # → (B, 512, 7, 7)
```

Note: truncate before the average pool (`[:-2]` not `[:-1]`) to preserve spatial information. The current model uses `[:-1]` which produces `(B, 512, 1, 1)` -- spatial info is needed for heatmap generation.

Freeze strategy: same as current model -- freeze layers before residual blocks 6 and 7.

### Decoder

Transposed convolution decoder upsamples from 7x7 to 56x56:

```python
decoder = nn.Sequential(
    nn.ConvTranspose2d(512, 256, kernel_size=4, stride=2, padding=1),  # 7 → 14
    nn.BatchNorm2d(256),
    nn.ReLU(),
    nn.ConvTranspose2d(256, 128, kernel_size=4, stride=2, padding=1),  # 14 → 28
    nn.BatchNorm2d(128),
    nn.ReLU(),
    nn.ConvTranspose2d(128, 7, kernel_size=4, stride=2, padding=1),    # 28 → 56
)
```

Output: `(B, 7, 56, 56)` -- one heatmap channel per keypoint.

### Soft-Argmax

Differentiable spatial expectation to extract (u, v) coordinates from heatmaps:

```python
def soft_argmax(heatmaps: torch.Tensor) -> torch.Tensor:
    """
    Args:
        heatmaps: (B, K, H, W) raw logits
    Returns:
        coords: (B, K, 2) normalized coordinates in [0, 1]
    """
    B, K, H, W = heatmaps.shape
    # Spatial softmax
    flat = heatmaps.view(B, K, -1)
    weights = torch.softmax(flat, dim=-1)
    weights = weights.view(B, K, H, W)

    # Coordinate grids [0, 1]
    y_grid = torch.linspace(0, 1, H, device=heatmaps.device)
    x_grid = torch.linspace(0, 1, W, device=heatmaps.device)
    yy, xx = torch.meshgrid(y_grid, x_grid, indexing="ij")

    # Weighted sum
    u = (weights * xx.unsqueeze(0).unsqueeze(0)).sum(dim=(-2, -1))  # (B, K)
    v = (weights * yy.unsqueeze(0).unsqueeze(0)).sum(dim=(-2, -1))  # (B, K)
    return torch.stack([u, v], dim=-1)  # (B, K, 2)
```

Soft-argmax is preferred over hard argmax because it is differentiable, enabling end-to-end training with coordinate loss.

### Loss Function

Combined heatmap + coordinate loss:

```python
loss = lambda_heatmap * MSE(pred_heatmaps, target_heatmaps) + lambda_coord * L1(pred_coords, target_coords)
```

- `lambda_heatmap = 1.0` -- primary supervision signal
- `lambda_coord = 5.0` -- coordinate regression loss for fine accuracy
- Target heatmaps: Gaussian blobs centered at ground-truth keypoint locations
  - Gaussian sigma = 2.0 pixels (in 56x56 heatmap space)
  - Peak value = 1.0, background = 0.0
- Target coordinates: normalized (u, v) in [0, 1] range

---

## Forward Kinematics Module

Computes 3D positions of the 7 keypoints given 5 joint angles, using the kinematic chain from `embodiment/v1/kinematics.json`.

### Kinematic Chain

Load at initialization from `kinematics.json`:

```python
chain = [
    # (parent_index, offset_xyz, axis_xyz)
    # Joint 1: base → link1, offset (0, 0, 0.04264), axis Z
    (0, [0.0, 0.0, 0.04264], [0, 0, 1]),
    # Joint 2: link1 → link2, offset (0, 0.02716, 0.06789), axis -Y
    (1, [0.0, 0.02716, 0.06789], [0, -1, 0]),
    # Joint 3: link2 → link3, offset (0, -0.01023, 0.09820), axis -Y
    (2, [0.0, -0.01023, 0.09820], [0, -1, 0]),
    # Joint 4: link3 → link4, offset (-0.01565, -0.01828, 0.10331), axis X
    (3, [-0.01565, -0.01828, 0.10331], [1, 0, 0]),
    # Joint 5: link4 → head, offset (0.01565, 0.0, 0.11544), axis Z
    (4, [0.01565, 0.0, 0.11544], [0, 0, 1]),
]
```

### FK Computation

```python
def forward_kinematics(joint_angles: np.ndarray) -> np.ndarray:
    """
    Args:
        joint_angles: (5,) array of joint angles in radians
    Returns:
        keypoints_3d: (7, 3) array of 3D keypoint positions in base frame
    """
    keypoints = np.zeros((7, 3))
    keypoints[0] = [0, 0, 0]  # base_center (fixed)

    T = np.eye(4)  # cumulative transform, base frame
    for i, (parent_idx, offset, axis) in enumerate(chain):
        # Translate to joint
        T_offset = np.eye(4)
        T_offset[:3, 3] = offset
        T = T @ T_offset

        # Rotate by joint angle
        angle = joint_angles[i]
        R = rotation_matrix(axis, angle)
        T_rot = np.eye(4)
        T_rot[:3, :3] = R
        T = T @ T_rot

        keypoints[i + 1] = T[:3, 3]  # Joint position (keypoints 1-5)

    # Keypoint 6: head_tip, fixed offset above J5
    T_head = np.eye(4)
    T_head[:3, 3] = [0, 0, 0.050]  # 50mm above J5
    T_final = T @ T_head
    keypoints[6] = T_final[:3, 3]

    return keypoints
```

Where `rotation_matrix(axis, angle)` returns a 3x3 rotation matrix using Rodrigues' formula for the given axis and angle.

---

## PnP Camera Pose Solver

### Purpose

Given detected 2D keypoints and known 3D keypoint positions (from FK at a known pose), solve for camera extrinsics (rotation and translation).

### Inputs

- `points_2d`: (N, 2) detected keypoint pixel coordinates (in original image space, e.g., 640x480)
- `points_3d`: (N, 3) corresponding 3D positions in the robot base frame
- `camera_matrix`: (3, 3) intrinsic matrix (focal length, principal point)
- `dist_coeffs`: (4,) or (5,) distortion coefficients (can be zeros for a reasonable webcam)

### Algorithm

```python
import cv2

success, rvec, tvec = cv2.solvePnP(
    points_3d,      # (N, 3) float64
    points_2d,      # (N, 2) float64
    camera_matrix,  # (3, 3) float64
    dist_coeffs,    # (4,) float64
    flags=cv2.SOLVEPNP_ITERATIVE,
)
```

`SOLVEPNP_ITERATIVE` (Levenberg-Marquardt) is used because:
- We have exactly 7 correspondences (sufficient for the 6-DOF pose)
- It handles the over-determined case well
- Robust initialization from `SOLVEPNP_SQPNP` can be used as a fallback

### Output

- `rvec`: (3, 1) Rodrigues rotation vector (camera-from-world)
- `tvec`: (3, 1) translation vector (camera-from-world)
- Conversion: `R, _ = cv2.Rodrigues(rvec)` gives the 3x3 rotation matrix

### Camera Intrinsics

Default intrinsics for a typical 640x480 webcam (to be refined during calibration):

```python
camera_matrix = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
])
dist_coeffs = np.zeros(4)
```

These can be overridden via the calibration file or by running OpenCV's `calibrateCamera` separately.

---

## Inverse Kinematics Solver

### Purpose

Given detected 2D keypoints and a known camera pose (from calibration), recover the 5 joint angles that best explain the observed keypoint positions.

### Method

Minimize reprojection error: find joint angles `q` such that the 2D projections of FK(q) through the known camera match the detected 2D keypoints.

```python
from scipy.optimize import minimize

def ik_solve(
    detected_2d: np.ndarray,       # (7, 2) detected keypoints in pixel coords
    camera_matrix: np.ndarray,      # (3, 3)
    rvec: np.ndarray,               # (3, 1) camera rotation
    tvec: np.ndarray,               # (3, 1) camera translation
    q_init: np.ndarray = None,      # (5,) initial guess (previous frame or zeros)
) -> np.ndarray:
    """Returns (5,) joint angles in radians."""

    if q_init is None:
        q_init = np.zeros(5)

    def cost(q):
        kp_3d = forward_kinematics(q)  # (7, 3)
        projected, _ = cv2.projectPoints(kp_3d, rvec, tvec, camera_matrix, None)
        projected = projected.squeeze()  # (7, 2)
        # Only minimize over movable keypoints (indices 2-6, affected by joints)
        error = projected[2:] - detected_2d[2:]
        return np.sum(error ** 2)

    # Joint limits from kinematics (radians)
    bounds = [
        (-1.2, 1.2),   # J1 yaw
        (-0.8, 1.2),   # J2 pitch
        (-1.0, 1.0),   # J3 pitch
        (-1.2, 1.2),   # J4 roll
        (-1.5, 1.5),   # J5 yaw
    ]

    result = minimize(cost, q_init, method="L-BFGS-B", bounds=bounds)
    return result.x
```

### Performance Notes

- Use the previous frame's joint angles as `q_init` for warm-starting (convergence in 3-5 iterations instead of 15-20).
- For the Rust inference pipeline, implement the IK solver analytically or use a fixed-iteration Gauss-Newton to avoid depending on a general optimizer. The 5-DOF serial chain with known camera pose is solvable in ~10 iterations of Gauss-Newton.
- Keypoints 0 and 1 (base_center, joint_1_top) are fixed and used only during calibration (PnP), not during IK.

### Base Orientation

Base roll and pitch are extracted from the camera-relative position of fixed keypoints (0 and 1). If the base tilts, the observed positions of these keypoints shift relative to the camera -- the deviation from the calibrated positions gives base_roll and base_pitch:

```python
def estimate_base_orientation(
    detected_2d: np.ndarray,   # (7, 2)
    calibrated_2d: np.ndarray, # (7, 2) keypoints at neutral pose from calibration
    camera_matrix: np.ndarray,
) -> tuple[float, float]:
    """Estimate base roll and pitch from fixed keypoint displacement."""
    # Use keypoints 0 and 1 (fixed on base)
    delta = detected_2d[:2] - calibrated_2d[:2]
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    # Approximate: pixel displacement / focal length ≈ angular displacement
    base_pitch = np.mean(delta[:, 1]) / fy  # vertical pixel shift → pitch
    base_roll = np.mean(delta[:, 0]) / fx   # horizontal pixel shift → roll
    return float(base_roll), float(base_pitch)
```

---

## Auto-Calibration Flow

The auto-calibration procedure determines camera extrinsics (pose relative to the robot base) from scratch, enabling placement of the camera at any arbitrary position without retraining.

### Prerequisites

- Keypoint model trained and deployed (ONNX, loaded in Rust inference)
- Camera intrinsics approximately known (default 600px focal length, or from prior calibration)
- Robot controller accessible via WebSocket at `ws://localhost:8765` (per `metak-shared/api-contracts/controller-websocket.md`)

### Steps

**Step 1: Neutral pose acquisition**
- Command robot to neutral pose (all joints = 0) via controller WebSocket
- Wait 1 second for servos to settle
- Run keypoint detection on current frame
- Validate: all 7 keypoints detected with confidence > 0.5 (heatmap peak > 0.5)

**Step 2: Initial camera pose estimate**
- Use detected 2D keypoints + known 3D positions at neutral pose
- Run `cv2.solvePnP` with `SOLVEPNP_SQPNP` (robust to initialization)
- Compute reprojection error; if > 10px, warn and retry with different PnP method

**Step 3: Refinement through commanded poses**
- Command robot through 5 calibration poses via controller WebSocket:
  - Pose 1: J1 = +0.5 rad (yaw left)
  - Pose 2: J1 = -0.5 rad (yaw right)
  - Pose 3: J2 = +0.5 rad (lean forward)
  - Pose 4: J3 = -0.5 rad (elbow bend)
  - Pose 5: J2 = +0.3, J3 = +0.3, J4 = +0.5 (combined)
- At each pose:
  - Wait 0.8 seconds for servos to settle
  - Detect 2D keypoints
  - Compute expected 3D keypoints via FK(commanded_angles)
  - Run PnP → get camera pose estimate for this pose

**Step 4: Refine camera extrinsics**
- Collect all (2D keypoint, 3D keypoint) correspondences from all 6 poses (neutral + 5 calibration)
- Total: 6 poses x 7 keypoints = 42 correspondences
- Run `cv2.solvePnP` on the full set (over-determined, robust) for a refined camera pose
- Alternatively, use bundle adjustment: minimize total reprojection error across all poses jointly, optimizing camera extrinsics + refining intrinsics (focal length)

**Step 5: Convergence check**
- Reproject all 42 points through the refined camera pose
- Compute mean reprojection error
- If mean error < 3.0 pixels: calibration succeeded
- If mean error > 3.0 pixels after 2 attempts: warn user, suggest better camera placement or lighting

**Step 6: Save calibration**
- Write `vimu_calibration.json` to the model directory:

```json
{
  "camera_matrix": [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
  "dist_coeffs": [0.0, 0.0, 0.0, 0.0],
  "rvec": [0.123, -0.456, 0.789],
  "tvec": [0.15, 0.10, 0.80],
  "reprojection_error_px": 1.8,
  "num_poses": 6,
  "timestamp": "2026-03-29T14:30:00Z"
}
```

- Switch to inference mode automatically upon successful calibration

### Calibration Duration

Target: < 30 seconds total (6 poses x ~2s each for command + settle + detect + solve, plus overhead).

---

## Training Data Requirements

### Extending Data Collection

Extend `vimu/training/collect.py` with keypoint ground-truth generation. During collection, joint angles are known (they are commanded), so 3D keypoints can be computed via FK and projected to 2D using the camera intrinsics.

Add a `--keypoints` flag to the `sweep` subcommand:

```
python collect.py sweep --ws ws://localhost:8765 \
    --camera 0 --num-joints 5 --num-poses 1500 --output-dir ./data \
    --keypoints --kinematics embodiment/v1/kinematics.json \
    --camera-matrix 600,600,320,240
```

This computes FK for each commanded pose, projects through the camera, and writes additional columns to `labels.csv`.

### Label Format

Extended `labels.csv` with keypoint columns appended:

```
frame, joint_1, ..., joint_5, base_roll, base_pitch, kp_0_x, kp_0_y, kp_1_x, kp_1_y, ..., kp_6_x, kp_6_y
000000.jpg, 0.52, ..., -0.10, 0.0, 0.0, 312.5, 421.0, 310.2, 385.3, ..., 298.1, 102.7
```

- `kp_N_x`, `kp_N_y`: pixel coordinates in the original image (640x480)
- 14 additional columns (7 keypoints x 2 coordinates)
- Keypoints that fall outside the image frame are marked as -1.0, -1.0 (occluded/OOB)

### Synthetic Data Extension

When Phase B synthetic data generation exists (`generate_synthetic_data.py` or Isaac Sim pipeline), extend it with:
- Render 3D keypoint positions from the simulation
- Project through the virtual camera to get 2D ground truth
- Apply the same domain randomization (viewpoint, lighting, background) to the keypoint labels

### Dataset Class

New `VimuKeypointDataset` in `vimu/training/dataset.py`:

```python
class VimuKeypointDataset(Dataset):
    """Dataset for keypoint detection training."""

    def __init__(
        self,
        data_dir: str,
        num_keypoints: int = 7,
        heatmap_size: int = 56,
        sigma: float = 2.0,
        transform=None,
    ):
        ...

    def _generate_heatmap(self, kp_x: float, kp_y: float) -> np.ndarray:
        """
        Generate a 56x56 Gaussian heatmap centered at (kp_x, kp_y).

        Args:
            kp_x, kp_y: keypoint coordinates normalized to [0, 1]
        Returns:
            (56, 56) float32 array, peak=1.0, sigma=2.0 pixels
        """
        heatmap = np.zeros((self.heatmap_size, self.heatmap_size), dtype=np.float32)
        if kp_x < 0 or kp_y < 0:  # occluded marker
            return heatmap

        cx = kp_x * self.heatmap_size
        cy = kp_y * self.heatmap_size

        for y in range(self.heatmap_size):
            for x in range(self.heatmap_size):
                heatmap[y, x] = np.exp(-((x - cx)**2 + (y - cy)**2) / (2 * self.sigma**2))
        return heatmap

    def __getitem__(self, idx):
        ...
        # Returns:
        # {
        #     "image": (3, 224, 224) tensor,
        #     "heatmaps": (7, 56, 56) tensor,
        #     "keypoints": (7, 2) tensor (normalized coords),
        #     "visibility": (7,) bool tensor (False if occluded/OOB),
        # }
```

---

## Rust Inference Pipeline Changes

### New File: `vimu/inference/src/keypoint.rs`

Contains the keypoint post-processing, PnP solver, and IK solver for the Rust inference path.

```rust
// vimu/inference/src/keypoint.rs

/// Extract 2D keypoint coordinates from heatmaps via soft-argmax.
pub fn soft_argmax(heatmaps: &[f32], num_kp: usize, h: usize, w: usize) -> Vec<(f32, f32)> { ... }

/// Solve camera pose from 2D-3D correspondences using OpenCV solvePnP.
pub fn solve_pnp(
    points_2d: &[(f32, f32)],
    points_3d: &[(f32, f32, f32)],
    camera_matrix: &[f64; 9],
) -> Option<(Vec<f64>, Vec<f64>)> { ... }  // (rvec, tvec)

/// Forward kinematics: joint angles → 3D keypoint positions.
pub fn forward_kinematics(joint_angles: &[f64], chain: &KinematicChain) -> Vec<[f64; 3]> { ... }

/// Inverse kinematics: minimize reprojection error.
/// Uses Gauss-Newton with warm start from previous frame.
pub fn ik_solve(
    detected_2d: &[(f32, f32)],
    camera_matrix: &[f64; 9],
    rvec: &[f64],
    tvec: &[f64],
    chain: &KinematicChain,
    q_init: &[f64],
    max_iters: usize,       // default: 10
) -> Vec<f64> { ... }       // 5 joint angles

/// Estimate base orientation from fixed keypoint displacement.
pub fn estimate_base_orientation(
    detected_2d: &[(f32, f32)],
    calibrated_2d: &[(f32, f32)],
    fx: f64,
    fy: f64,
) -> (f64, f64) { ... }     // (base_roll, base_pitch)
```

### New File: `vimu/inference/src/calibration.rs`

Calibration mode, triggered by CLI flag `--calibrate`:

```rust
// vimu/inference/src/calibration.rs

pub struct CalibrationConfig {
    pub controller_ws_url: String,        // e.g., "ws://localhost:8765"
    pub num_calibration_poses: usize,     // default: 5
    pub settle_time_ms: u64,              // default: 800
    pub convergence_threshold_px: f64,    // default: 3.0
    pub output_path: String,              // e.g., "./vimu_calibration.json"
}

/// Run the auto-calibration procedure.
/// Connects to controller WebSocket, commands poses, detects keypoints,
/// solves PnP, and saves calibration file.
pub fn run_calibration(
    config: &CalibrationConfig,
    model: &mut Model,
    camera: &mut Camera,
) -> Result<CalibrationResult> { ... }
```

### Cargo.toml Changes

Add `calib3d` to the opencv features in `vimu/inference/Cargo.toml`:

```toml
# Current:
opencv = { version = "0.98", features = ["videoio", "imgproc", "highgui"], optional = true }

# Updated:
opencv = { version = "0.98", features = ["videoio", "imgproc", "highgui", "calib3d"], optional = true }
```

Add `websocket-client` dependency for calibration mode (connecting to controller):

```toml
tungstenite = "0.24"  # sync WebSocket client for calibration
```

Note: `tokio-tungstenite` is already a dependency for the WS server. For calibration, a synchronous client (`tungstenite`) is simpler since calibration runs sequentially.

### model.rs Changes

Extend `ModelMeta` to support both model types:

```rust
#[derive(Deserialize, Clone)]
pub struct ModelMeta {
    pub num_joints: usize,
    pub output_dim: usize,
    pub outputs: Vec<String>,
    // New fields (optional for backward compat with regression models)
    #[serde(default)]
    pub model_type: String,        // "regression" (default) or "keypoint"
    #[serde(default)]
    pub num_keypoints: Option<usize>,   // 7
    #[serde(default)]
    pub heatmap_size: Option<usize>,    // 56
}
```

The `Model::predict` method remains unchanged -- it returns raw ONNX output. The interpretation differs based on `model_type`:
- `"regression"`: output is `(1, 7)` -- direct joint angles + base state (current behavior)
- `"keypoint"`: output is `(1, 7, 56, 56)` -- 7 heatmap channels

### pipeline.rs Changes

After model prediction, branch based on model type:

```rust
// In run() main loop, after step 2 (infer):

let (joint_angles, base_roll, base_pitch) = match model.meta.model_type.as_str() {
    "keypoint" => {
        // Reshape output to (7, 56, 56) heatmaps
        let heatmaps = &raw;  // raw is Vec<f32> of length 7*56*56
        let kp_2d = keypoint::soft_argmax(heatmaps, 7, 56, 56);

        // Scale from [0,1] to pixel coordinates
        let kp_2d_px: Vec<(f32, f32)> = kp_2d.iter()
            .map(|(u, v)| (u * img_width as f32, v * img_height as f32))
            .collect();

        // IK solve using calibrated camera pose
        let joints = keypoint::ik_solve(
            &kp_2d_px, &calib.camera_matrix, &calib.rvec, &calib.tvec,
            &chain, &prev_joints, 10,
        );

        // Base orientation from fixed keypoints
        let (roll, pitch) = keypoint::estimate_base_orientation(
            &kp_2d_px, &calib.neutral_keypoints, calib.fx, calib.fy,
        );

        (joints, roll, pitch)
    }
    _ => {
        // Current regression behavior: raw[0..5] are joints, raw[5..7] are base
        let joints = raw[..5].to_vec();
        (joints, raw[5] as f64, raw[6] as f64)
    }
};

// Build measurement vector for EKF (same format as before)
let measurement: Vec<f64> = joint_angles.iter()
    .chain(std::iter::once(&base_roll))
    .chain(std::iter::once(&base_pitch))
    .copied()
    .collect();
```

The EKF and WebSocket broadcast remain unchanged -- they receive the same 7-dimensional measurement vector `[j1, j2, j3, j4, j5, base_roll, base_pitch]` regardless of model type.

### main.rs Changes

Add CLI flags:

```rust
#[derive(clap::Parser)]
struct Cli {
    // ... existing flags ...

    /// Run auto-calibration before inference
    #[arg(long)]
    calibrate: bool,

    /// Controller WebSocket URL (for calibration)
    #[arg(long, default_value = "ws://localhost:8765")]
    controller_ws: String,

    /// Path to calibration file (loaded at startup if exists)
    #[arg(long, default_value = "./vimu_calibration.json")]
    calibration_file: String,
}
```

Startup flow:
1. Load model and check `model_type` in metadata
2. If `model_type == "keypoint"`:
   a. If `--calibrate` flag: run calibration procedure, save file, then continue to inference
   b. Else: load `vimu_calibration.json` from `--calibration-file` path
   c. If calibration file missing: error with message to run with `--calibrate` first
3. If `model_type == "regression"`: run current pipeline unchanged

---

## ONNX Export Changes

Update `vimu/training/export_onnx.py` to handle the keypoint model:

### Output Shape

- Regression model (current): `(batch, 7)` -- 5 joints + 2 base
- Keypoint model (new): `(batch, 7, 56, 56)` -- 7 heatmap channels at 56x56 resolution

### Export Code

```python
# In export_onnx.py, after loading checkpoint:

if ckpt.get("model_type") == "keypoint":
    model = VimuKeypointModel(num_keypoints=ckpt["num_keypoints"])
    dummy = torch.randn(1, 3, 224, 224)
    # Output: (1, 7, 56, 56) heatmaps
    torch.onnx.export(
        model, dummy, output_path,
        input_names=["image"],
        output_names=["heatmaps"],
        dynamic_axes={"image": {0: "batch"}, "heatmaps": {0: "batch"}},
        opset_version=17,
    )
else:
    # Existing regression export (unchanged)
    ...
```

### Metadata

Extended `vimu.json` for keypoint models:

```json
{
  "model_type": "keypoint",
  "num_joints": 5,
  "num_keypoints": 7,
  "heatmap_size": 56,
  "output_dim": 7,
  "input_size": 224,
  "outputs": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "base_roll", "base_pitch"],
  "keypoint_names": ["base_center", "joint_1_top", "joint_2", "joint_3", "joint_4", "joint_5", "head_tip"],
  "imagenet_mean": [0.485, 0.456, 0.406],
  "imagenet_std": [0.229, 0.224, 0.225]
}
```

For backward compatibility, `model_type` defaults to `"regression"` when absent.

---

## File Plan

### New Files

| File | Description |
|------|-------------|
| `vimu/training/keypoint_model.py` | `VimuKeypointModel` class: ResNet-18 encoder + transposed-conv decoder + soft-argmax |
| `vimu/training/forward_kinematics.py` | FK computation from kinematic chain, used by data collection and training |
| `vimu/training/train_keypoints.py` | Training script for keypoint model (heatmap + coord loss) |
| `vimu/inference/src/keypoint.rs` | Soft-argmax, PnP solver, FK, IK solver (Rust) |
| `vimu/inference/src/calibration.rs` | Auto-calibration procedure (Rust) |

### Modified Files

| File | Changes |
|------|---------|
| `vimu/training/collect.py` | Add `--keypoints` flag, FK projection, extra CSV columns |
| `vimu/training/dataset.py` | Add `VimuKeypointDataset` class with heatmap target generation |
| `vimu/training/export_onnx.py` | Handle `model_type == "keypoint"`, export heatmap output, extended metadata |
| `vimu/inference/Cargo.toml` | Add `calib3d` to opencv features, add `tungstenite` dependency |
| `vimu/inference/src/model.rs` | Extend `ModelMeta` with `model_type`, `num_keypoints`, `heatmap_size` fields |
| `vimu/inference/src/pipeline.rs` | Branch after prediction: heatmap path (soft-argmax → PnP/IK) vs regression path |
| `vimu/inference/src/main.rs` | Add `--calibrate`, `--controller-ws`, `--calibration-file` CLI flags |
| `metak-shared/api-contracts/onnx-model-contract.md` | Add keypoint model variant to ONNX contract documentation |

### Unchanged Files

| File | Reason |
|------|--------|
| `vimu/inference/src/ekf.rs` | EKF receives same 7-dim measurement vector regardless of model type |
| `vimu/inference/src/ws.rs` | WebSocket broadcast format unchanged |
| `vimu/inference/src/camera.rs` | Camera capture unchanged |
| `projects/adelino/deployment/policy_runner.py` | Consumes VIMU WebSocket (unchanged contract) |
| `projects/adelino/deployment/observation_builder.py` | Unchanged |

---

## Acceptance Criteria

| Criterion | Metric | Target |
|-----------|--------|--------|
| Keypoint detection accuracy | Mean pixel error on synthetic validation set (56x56 heatmap space) | < 5 px (< 2 px in 224x224 image space) |
| PnP camera pose accuracy | Angular error vs ground truth on synthetic data | < 5 degrees |
| PnP translation accuracy | Translation error vs ground truth | < 20 mm |
| IK joint angle accuracy | Mean absolute error on synthetic data with known camera pose | < 3 degrees per joint |
| Auto-calibration convergence | Time from start to saved calibration file | < 30 seconds |
| Auto-calibration accuracy | Mean reprojection error after calibration | < 3.0 pixels |
| Inference latency | End-to-end frame time (capture + model + post-processing) | < 16 ms (60+ FPS) on CUDA |
| Backward compatibility | Regression models load and run without code changes | Pass existing tests |
| Camera placement flexibility | Works from any position 0.5-1.5m from robot, 15-165 deg elevation | Qualitative |

---

## Risks and Mitigations

### 1. OpenCV calib3d Rust Bindings on Windows

**Risk**: The `opencv` Rust crate's `calib3d` module may have build issues on Windows, especially with `solvePnP`.

**Mitigation**:
- Test `calib3d` feature compilation early as a spike
- Fallback: implement PnP solver in pure Rust using `nalgebra` (already a dependency). The DLT (Direct Linear Transform) algorithm + iterative refinement is straightforward for 7+ correspondences
- Alternative: use the `cv` crate which has simpler FFI

### 2. Heatmap Output Size Impact on Inference Latency

**Risk**: The keypoint model outputs `(7, 56, 56)` = 21,952 floats vs the regression model's 7 floats. The decoder adds transposed convolutions.

**Mitigation**:
- 56x56 heatmaps are small; the transposed conv decoder adds < 1ms on GPU
- Soft-argmax is trivially fast (weighted sum)
- The PnP and IK solvers operate on 7 points -- negligible compute
- Profile the full pipeline; if > 16ms, reduce heatmap size to 28x28 (sigma=1.0)

### 3. PnP Convergence with Noisy Keypoints

**Risk**: Noisy keypoint detections (jitter, partial occlusion) cause PnP to produce unstable camera pose estimates, which cascade into noisy IK solutions.

**Mitigation**:
- During calibration: average PnP solutions across 6+ poses (over-determined system is robust)
- During inference: camera pose is fixed (loaded from calibration file), only IK runs per-frame
- The EKF already filters noisy measurements -- IK jitter is smoothed the same way regression noise was
- Add visibility/confidence filtering: ignore keypoints with heatmap peak < 0.3

### 4. Keypoint Occlusion

**Risk**: Some keypoints may be occluded from certain camera angles (e.g., base_center hidden behind the robot body).

**Mitigation**:
- PnP needs minimum 4 non-coplanar points; with 7 keypoints, losing 1-2 is acceptable
- Train with occlusion augmentation: randomly zero out 1-2 keypoint heatmaps during training
- At inference, filter by heatmap confidence and only use visible keypoints for PnP/IK
- Keypoint 6 (head_tip) is almost always visible from any angle

### 5. Sim-to-Real Gap for Keypoint Detection

**Risk**: A model trained only on real data from one camera angle may not generalize. A model trained on synthetic data may not transfer to real images.

**Mitigation**:
- Phase B (synthetic pre-training with domain randomization) provides viewpoint-diverse training data
- Fine-tune on a small real dataset (100-200 frames from the target camera)
- Keypoint detection is inherently more transferable than direct regression because it learns local visual features (joint markers/edges) rather than a global image-to-angle mapping

### 6. Camera Intrinsics Uncertainty

**Risk**: Default focal length (600px) may be inaccurate, causing systematic PnP errors.

**Mitigation**:
- Auto-calibration Step 4 can optionally refine focal length (add fx, fy to the optimization)
- Provide a separate `--calibrate-intrinsics` mode using OpenCV's `calibrateCamera` with a checkerboard
- Even 10% focal length error produces < 2 degree joint angle error at typical viewing distances (0.5-1.0m)
