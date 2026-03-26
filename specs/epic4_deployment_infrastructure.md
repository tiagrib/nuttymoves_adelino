# Epic 4: Deployment Infrastructure

**Type**: DevOps / Infrastructure
**Status**: Not Started
**Priority**: P2
**Depends On**: Epic 3 (Training Pipeline) - at least Task 1 trained

---

## Overview

Set up the infrastructure for deploying trained policies to the physical Adelino robot. This includes Arduino firmware, Python control interface, and the complete deployment pipeline.

## Goals

- Arduino firmware for servo control and IMU reading
- Python interface for policy execution
- Real-time control loop (30-50 Hz)
- Calibration and testing tools
- Documentation for hardware setup

---

## Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | Arduino Mega 2560 | Servo PWM, IMU I2C, Serial comms |
| IMU | BNO085 | Orientation sensing (quaternion output) |
| Servos (5) | HobbyKing HK15xxx | Joint actuation |
| Power | 6V 5A supply | Servo power (separate from Arduino) |
| USB Cable | USB-B | Serial connection to PC |

---

## Tasks

### Task 4.1: Design Communication Protocol
**Status**: Not Started

Define serial protocol between Python and Arduino:

```
# Python -> Arduino (Command)
CMD:<j1>,<j2>,<j3>,<j4>,<j5>\n
# Values: PWM microseconds (1000-2000)

# Arduino -> Python (State)
STATE:<q_w>,<q_x>,<q_y>,<q_z>,<p1>,<p2>,<p3>,<p4>,<p5>\n
# q_*: IMU quaternion
# p_*: Servo positions (feedback if available)
```

**Acceptance Criteria**:
- Protocol documented
- Error handling defined
- Baud rate specified (115200)

---

### Task 4.2: Create Arduino Firmware
**Status**: Not Started

- [ ] Initialize servo library for 5 servos
- [ ] Initialize I2C for BNO085 IMU
- [ ] Parse incoming commands
- [ ] Send state updates at fixed rate
- [ ] Implement safety limits

**Output**: `deployment/arduino/adelino_firmware/adelino_firmware.ino`

**Key Features**:
- 50Hz control loop
- PWM limits enforced in firmware
- Watchdog timeout (stop servos if no commands)
- LED status indicator

**Acceptance Criteria**:
- Compiles without errors
- Servos respond to commands
- IMU data reads correctly
- Safety limits work

---

### Task 4.3: Create Python Control Interface
**Status**: Not Started

- [ ] Serial communication class
- [ ] State parsing and validation
- [ ] Command sending with rate limiting
- [ ] Reconnection handling

**Output**: `deployment/python/robot_interface.py`

```python
class AdelinoInterface:
    def __init__(self, port: str, baud: int = 115200):
        ...

    def get_state(self) -> RobotState:
        """Returns current IMU orientation and joint positions"""
        ...

    def set_joints(self, positions: np.ndarray):
        """Send joint position commands (radians)"""
        ...

    def close(self):
        ...
```

**Acceptance Criteria**:
- Connects reliably
- State updates at expected rate
- Commands sent correctly
- Handles disconnection gracefully

---

### Task 4.4: Create Policy Runner
**Status**: Not Started

- [ ] Load TorchScript policy
- [ ] Construct observations from robot state
- [ ] Run policy inference
- [ ] Send actions to robot
- [ ] Main control loop with timing

**Output**: `deployment/python/policy_runner.py`

```python
class PolicyRunner:
    def __init__(self, policy_path: str, robot: AdelinoInterface):
        self.policy = torch.jit.load(policy_path)
        self.robot = robot

    def run(self, duration: float = 30.0):
        """Run policy for specified duration"""
        while time.time() < start + duration:
            state = self.robot.get_state()
            obs = self.construct_observation(state)
            action = self.policy(obs)
            self.robot.set_joints(action)
            # Rate limiting to 30-50 Hz
```

**Acceptance Criteria**:
- Runs at target frequency
- Observations constructed correctly
- Actions applied smoothly
- Graceful shutdown

---

### Task 4.5: Create Calibration Tools
**Status**: Not Started

- [ ] Servo center calibration script
- [ ] IMU alignment verification
- [ ] Joint limit testing
- [ ] Servo PWM mapping verification

**Output**: `deployment/python/calibration.py`

**Calibration Procedure**:
1. Move each servo to mechanical center
2. Record PWM value at center
3. Verify range of motion matches simulation
4. Align IMU reference frame

**Acceptance Criteria**:
- Calibration values saved to config
- Repeatable process documented
- Warnings if limits exceeded

---

### Task 4.6: Create Testing Tools
**Status**: Not Started

- [ ] Individual servo test
- [ ] IMU data visualization
- [ ] Joint trajectory playback
- [ ] Safety stop test

**Output**: `deployment/python/test_hardware.py`

**Acceptance Criteria**:
- Can test each component independently
- Visual feedback for debugging
- Safety stop works reliably

---

### Task 4.7: Create Deployment Script
**Status**: Not Started

One-command deployment:

```powershell
python deployment/python/run_policy.py \
    --policy models/orientation_policy.pt \
    --port COM3 \
    --duration 60
```

**Output**: `deployment/python/run_policy.py`

**Acceptance Criteria**:
- Single command to run policy
- Logging of run data
- Clean shutdown on Ctrl+C

---

### Task 4.8: Document Hardware Setup
**Status**: Not Started

- [ ] Wiring diagram
- [ ] Power requirements
- [ ] Arduino pin assignments
- [ ] IMU mounting orientation
- [ ] Troubleshooting guide

**Output**: Deployment documentation (location TBD)

**Acceptance Criteria**:
- Complete wiring documented
- Photos/diagrams included
- Common issues addressed

---

## File Structure

```
deployment/
├── arduino/
│   └── adelino_firmware/
│       ├── adelino_firmware.ino   # Main firmware
│       ├── config.h               # Pin assignments, limits
│       └── README.md              # Upload instructions
├── python/
│   ├── robot_interface.py         # Hardware communication
│   ├── policy_runner.py           # Policy execution
│   ├── calibration.py             # Calibration tools
│   ├── test_hardware.py           # Hardware testing
│   ├── run_policy.py              # Main entry point
│   └── config/
│       └── robot_config.yaml      # Calibration values
└── models/
    └── (exported policies)
```

---

## Pin Assignments (Arduino Mega)

| Pin | Function |
|-----|----------|
| 2 | Servo J1 (Base Yaw) |
| 3 | Servo J2 (Pitch) |
| 4 | Servo J3 (Pitch) |
| 5 | Servo J4 (Roll) |
| 6 | Servo J5 (Head) |
| 20 (SDA) | IMU I2C Data |
| 21 (SCL) | IMU I2C Clock |
| 13 | Status LED |

---

## Safety Considerations

1. **Servo Power**: Use separate 6V supply, NOT Arduino 5V
2. **Current**: Ensure supply can handle 5 servos (~3A peak)
3. **Watchdog**: Firmware stops servos if no commands for 500ms
4. **PWM Limits**: Enforce 1000-2000μs range in firmware
5. **Software E-Stop**: Ctrl+C cleanly returns to neutral pose

---

## Completion Criteria

- [ ] Arduino firmware uploaded and tested
- [ ] Python interface communicates reliably
- [ ] Calibration completed
- [ ] At least one policy runs on real robot
- [ ] Hardware setup documented
- [ ] Safety features verified

---

## Notes

- Start with simple tests (single servo, IMU only) before full integration
- Log all deployment runs for analysis
- Expect sim-to-real gap - may need observation noise or domain randomization in training
- Consider adding video recording for analysis
