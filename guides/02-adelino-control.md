# Adelino Controller Guide

Step-by-step instructions for setting up, calibrating, and controlling the Adelino 5-DOF servo robot.

## 1. Hardware Setup

**Components:**
- Arduino Mega 2560
- 5 hobby servos wired to digital pins 2-6:
  - Pin 2: J1 -- Base Yaw (HK15338)
  - Pin 3: J2 -- Pitch (HK15298B)
  - Pin 4: J3 -- Pitch (HK15328A)
  - Pin 5: J4 -- Roll (HK15138)
  - Pin 6: J5 -- Head Yaw (HK15178)
- Separate 6V power supply for servos
- USB-B cable connecting Arduino to PC

**Important:** Do NOT power servos from the Arduino's 5V pin. Servo stall current can exceed what the Arduino can supply and may damage the board. Use a dedicated 6V power supply with its ground connected to the Arduino's GND.

## 2. Prerequisites

**Working directory:** All `adelino-standalone` commands in this guide assume you are in the `projects/adelino/` directory (the Adelino Cargo workspace root).

**Build the binary:**

```
cd projects/adelino
cargo build --release -p adelino-standalone
```

You can then run commands in two ways:

```
# Option A: via cargo (rebuilds if source changed)
cargo run --release -p adelino-standalone -- <subcommand> [args]

# Option B: run the built binary directly
./target/release/adelino-standalone <subcommand> [args]
```

All examples below use the short form (`adelino-standalone ...`) for readability. Substitute whichever method you prefer.

**Rust toolchain:** Install from [rustup.rs](https://rustup.rs/) if `cargo` is not available. After installing, you may need to restart your terminal (or relaunch VSCode from a fresh shell) for PATH changes to take effect.

## 3. Install arduino-cli (for flashing)

Download and install from the [official arduino-cli documentation](https://arduino.github.io/arduino-cli/latest/installation/).

After installation, install the AVR core:

```
arduino-cli core install arduino:avr
```

Verify:

```
arduino-cli version
```

## 4. Flash Firmware

With the Arduino connected via USB:

```
adelino-standalone flash --port COM3
```

This compiles and uploads the firmware from `firmware/` to the Arduino Mega 2560.

The default `--fqbn` is `arduino:avr:mega:cpu=atmega2560`. If using a different board, override it:

```
adelino-standalone flash --port COM3 --fqbn arduino:avr:mega:cpu=atmega2560
```

If the firmware directory is not found automatically, specify it:

```
adelino-standalone flash --port COM3 --firmware-dir path/to/projects/adelino/firmware
```

## 5. Run the Controller

Start the WebSocket-to-serial bridge:

```
adelino-standalone run --port COM3 --ws-port 8765 --baud 115200
```

This starts a WebSocket server on port 8765 that accepts JSON commands and forwards them to the Arduino over serial at 115200 baud. The controller runs a 50Hz control loop.

If you have a calibration file (see step 6), load it:

```
adelino-standalone run --port COM3 --ws-port 8765 --baud 115200 --calibration calibration.toml
```

If a `calibration.toml` file exists in the current directory, it will be loaded automatically even without the `--calibration` flag.

## 6. Calibrate Servos

Calibration maps radians to PWM microseconds for each servo, accounting for mechanical offsets and direction.

```
adelino-standalone calibrate --port COM3 --output calibration.toml
```

The interactive calibration process has four steps per joint:

1. **Center finding** -- Use `+`/`-` keys to nudge the servo PWM value (default step: 10us). Type a number for absolute PWM. Press Enter when the servo is at its mechanical center.

2. **Positive range** -- From center, nudge toward the positive physical limit. Press Enter at the safe maximum.

3. **Negative range** -- From center, nudge toward the negative physical limit. Press Enter at the safe minimum.

4. **Direction check** -- The servo moves to +0.1 rad. Answer `y` if this is the positive direction, `n` if inverted.

After calibrating all 5 joints, the result is saved to the specified TOML file. Use it with the `run` command:

```
adelino-standalone run --port COM3 --calibration calibration.toml
```

## 7. Test Servos

Test all servos by sweeping through their calibrated range:

```
adelino-standalone test --port COM3
```

Test a single joint (1-indexed):

```
adelino-standalone test --port COM3 --joint 2
```

Each joint sweeps: center -> max -> center -> min -> center, with 800ms between positions. If a calibration file exists, it will be used for accurate range limits.

Load a specific calibration for testing:

```
adelino-standalone test --port COM3 --calibration calibration.toml
```

## 8. WebSocket Protocol

The controller speaks the WebSocket JSON protocol defined in `metak-shared/api-contracts/controller-websocket.md`.

### Sending Commands

Send joint position commands as radians (ordered J1 through J5):

```python
import websocket
import json

ws = websocket.WebSocket()
ws.connect("ws://localhost:8765")

# Command all joints to zero (neutral)
ws.send(json.dumps({
    "type": "command",
    "positions": [0.0, 0.0, 0.0, 0.0, 0.0]
}))

# Read back the state
response = json.loads(ws.recv())
print(response)
# {
#   "type": "state",
#   "positions": [0.0, 0.0, 0.0, 0.0, 0.0],
#   "imu": null,
#   "status": {"imu_valid": false, "watchdog_active": false},
#   "timestamp_ms": 1234567890
# }

ws.close()
```

### Continuous Control Example

```python
import websocket
import json
import time
import math

ws = websocket.WebSocket()
ws.connect("ws://localhost:8765")

try:
    for i in range(300):
        t = i * 0.033  # ~30Hz
        positions = [
            0.3 * math.sin(t),       # J1: oscillate base yaw
            0.0,                       # J2: static
            0.0,                       # J3: static
            0.0,                       # J4: static
            0.2 * math.sin(t * 2),    # J5: oscillate head yaw
        ]
        ws.send(json.dumps({"type": "command", "positions": positions}))
        time.sleep(0.033)
finally:
    # Return to neutral before disconnecting
    ws.send(json.dumps({"type": "command", "positions": [0.0, 0.0, 0.0, 0.0, 0.0]}))
    time.sleep(0.1)
    ws.close()
```

### Protocol Details

- **Multiple clients**: The server accepts multiple simultaneous WebSocket connections. State is broadcast to all connected clients.
- **Watchdog**: If no command is received within 500ms, the controller sends a neutral pose to the Arduino. The `status.watchdog_active` field in the state message indicates when this has triggered.
- **Joint order**: Positions are always ordered J1 through J5, matching pins 2 through 6.

## 9. Troubleshooting

### Wrong COM port

**Symptom:** "Failed to open serial port" or similar error.

**Fix:** Check which port the Arduino is on:
- Windows: Device Manager -> Ports (COM & LPT)
- Use the correct `--port COMx` argument.

### Servos not moving

**Symptom:** Controller starts, WebSocket connects, but servos do not respond.

**Possible causes:**
- **No external power:** Servos need a separate 6V supply. Check that the power supply is on and the ground is shared with the Arduino.
- **Wrong pins:** Verify servos are connected to pins 2-6 as defined in `config.h`.
- **Firmware not flashed:** Re-flash with `adelino-standalone flash --port COM3`.

### Watchdog timeout

**Symptom:** State messages show `"watchdog_active": true`, servos return to neutral.

**Fix:** The controller expects commands at least every 500ms. Ensure your control loop is sending commands at a sufficient rate. If you are only reading state without sending commands, the watchdog will engage.

### Servo jitter

**Symptom:** Servos vibrate or oscillate around target position.

**Possible causes:**
- **Insufficient power supply current.** Each servo can draw 500mA+. Ensure your power supply can handle the total load.
- **USB cable too long or poor quality.** Use a short, shielded USB-B cable.
- **Calibration needed.** Run `adelino-standalone calibrate` to properly map PWM ranges.

### Serial communication errors

**Symptom:** "Frame checksum mismatch" or garbled data in logs.

**Fix:**
- Ensure baud rate matches: `--baud 115200` (default) must match the firmware's `SERIAL_BAUD` (115200 in `config.h`).
- Close any other programs using the same COM port (Serial Monitor, etc.).
- Try a different USB cable or port.
