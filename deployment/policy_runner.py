"""
Adelino Policy Runner — the sim-to-real bridge.

Reads proprioceptive state from VIMU inference (WebSocket),
runs the trained balance policy, and sends joint commands to
the robot controller (WebSocket).

Architecture:
    VIMU inference ──WS:9001──> [this process] ──WS:8765──> adelino-standalone ──serial──> Arduino

Usage:
    python policy_runner.py \
        --policy ../logs/rsl_rl/adelino_balance_flat/exported/policy.onnx \
        --vimu-ws ws://localhost:9001 \
        --controller-ws ws://localhost:8765 \
        --rate 30

Prerequisites:
    1. VIMU inference running:    vimu --model vimu.onnx --camera 0 --port 9001
    2. Controller running:        adelino-standalone run --port COM3 --ws-port 8765
"""

import argparse
import json
import signal
import sys
import time

import numpy as np

from config import (
    VIMU_WS_URL,
    CONTROLLER_WS_URL,
    CONTROL_RATE_HZ,
    ACTION_SCALE,
    NUM_JOINTS,
    DEFAULT_JOINT_POS,
    OBS_DIM,
)
from observation_builder import ObservationBuilder


class PolicyRunner:
    """Runs the trained policy in a real-time control loop."""

    def __init__(self, policy_path: str, vimu_url: str, controller_url: str, rate_hz: int):
        self.rate_hz = rate_hz
        self.obs_builder = ObservationBuilder()
        self.running = False

        # Load policy
        self.policy = self._load_policy(policy_path)
        print(f"Policy loaded: {policy_path}")

        # Connect to VIMU
        import websocket
        self.vimu_ws = websocket.WebSocket()
        self.vimu_ws.settimeout(0.1)  # Non-blocking reads
        print(f"Connecting to VIMU at {vimu_url}...")
        self.vimu_ws.connect(vimu_url)
        print(f"Connected to VIMU")

        # Connect to controller
        self.ctrl_ws = websocket.WebSocket()
        self.ctrl_ws.settimeout(1.0)
        print(f"Connecting to controller at {controller_url}...")
        self.ctrl_ws.connect(controller_url)
        print(f"Connected to controller")

    def _load_policy(self, path: str):
        """Load policy from ONNX or TorchScript."""
        if path.endswith(".onnx"):
            import onnxruntime as ort
            session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
            input_name = session.get_inputs()[0].name

            def run_policy(obs: np.ndarray) -> np.ndarray:
                obs_batch = obs.reshape(1, -1).astype(np.float32)
                result = session.run(None, {input_name: obs_batch})
                return result[0][0]  # Remove batch dim

            return run_policy

        elif path.endswith(".pt") or path.endswith(".jit"):
            import torch
            model = torch.jit.load(path, map_location="cpu")
            model.eval()

            @torch.no_grad()
            def run_policy(obs: np.ndarray) -> np.ndarray:
                obs_tensor = torch.from_numpy(obs).unsqueeze(0).float()
                action = model(obs_tensor)
                return action.squeeze(0).numpy()

            return run_policy

        else:
            raise ValueError(f"Unsupported policy format: {path} (expected .onnx or .pt)")

    def run(self, duration: float = None):
        """
        Main control loop.

        Args:
            duration: Run for this many seconds. None = run until Ctrl+C.
        """
        self.running = True
        period = 1.0 / self.rate_hz
        start_time = time.time()
        step_count = 0
        last_vimu_state = None

        print(f"\n=== Policy Runner started ({self.rate_hz} Hz) ===")
        print("Press Ctrl+C to stop\n")

        try:
            while self.running:
                loop_start = time.time()

                if duration and (loop_start - start_time) >= duration:
                    print(f"Duration {duration}s reached")
                    break

                # 1. Read latest VIMU state (drain to get most recent)
                vimu_state = self._read_latest_vimu()
                if vimu_state is not None:
                    last_vimu_state = vimu_state

                if last_vimu_state is None:
                    # No VIMU data yet, wait
                    if step_count == 0:
                        print("Waiting for VIMU data...")
                    time.sleep(0.01)
                    continue

                # 2. Build observation
                obs = self.obs_builder.build(last_vimu_state)

                # 3. Run policy
                raw_action = self.policy(obs)

                # 4. Scale action and apply as position offset from default
                action = np.clip(raw_action, -1.0, 1.0)
                target_positions = np.array(DEFAULT_JOINT_POS) + action * ACTION_SCALE
                self.obs_builder.update_actions(action)

                # 5. Send to controller
                self._send_command(target_positions.tolist())

                step_count += 1

                # Status logging
                if step_count % (self.rate_hz * 5) == 0:
                    elapsed = time.time() - start_time
                    actual_hz = step_count / elapsed if elapsed > 0 else 0
                    print(f"  Step {step_count} | {actual_hz:.1f} Hz | {elapsed:.1f}s")

                # Rate limiting
                elapsed = time.time() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            print("\nShutdown requested")
        finally:
            # Return to neutral
            print("Returning to neutral pose...")
            self._send_command(DEFAULT_JOINT_POS)
            time.sleep(0.2)
            self.vimu_ws.close()
            self.ctrl_ws.close()
            print(f"Done. Ran {step_count} steps.")

    def _read_latest_vimu(self) -> dict:
        """Read and return the most recent VIMU state, draining the buffer."""
        latest = None
        try:
            while True:
                msg = self.vimu_ws.recv()
                latest = json.loads(msg)
        except Exception:
            pass  # Timeout or empty = no more messages
        return latest

    def _send_command(self, positions: list):
        """Send joint position command to the controller."""
        msg = json.dumps({
            "type": "command",
            "positions": positions,
        })
        try:
            self.ctrl_ws.send(msg)
        except Exception as e:
            print(f"  WARNING: Controller send failed: {e}")

    def stop(self):
        self.running = False


def main():
    parser = argparse.ArgumentParser(description="Adelino policy runner (sim-to-real)")
    parser.add_argument("--policy", required=True,
                        help="Path to exported policy (.onnx or .pt)")
    parser.add_argument("--vimu-ws", default=VIMU_WS_URL,
                        help=f"VIMU inference WebSocket URL (default: {VIMU_WS_URL})")
    parser.add_argument("--controller-ws", default=CONTROLLER_WS_URL,
                        help=f"Controller WebSocket URL (default: {CONTROLLER_WS_URL})")
    parser.add_argument("--rate", type=int, default=CONTROL_RATE_HZ,
                        help=f"Control loop rate in Hz (default: {CONTROL_RATE_HZ})")
    parser.add_argument("--duration", type=float, default=None,
                        help="Run for N seconds (default: indefinite)")
    args = parser.parse_args()

    runner = PolicyRunner(
        policy_path=args.policy,
        vimu_url=args.vimu_ws,
        controller_url=args.controller_ws,
        rate_hz=args.rate,
    )

    # Handle SIGINT gracefully
    signal.signal(signal.SIGINT, lambda *_: runner.stop())

    runner.run(duration=args.duration)


if __name__ == "__main__":
    main()
