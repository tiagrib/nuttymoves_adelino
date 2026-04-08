// ============================================================
// Adelino Robot - Arduino Mega 2560 Firmware
// ============================================================
//
// Receives servo position commands over serial (binary protocol),
// drives 5 hobby servos, optionally reads BNO085 IMU, and
// sends state packets back at a fixed rate.
//
// Protocol: see protocol.h for packet format details.

#include <Servo.h>
#include "config.h"
#include "protocol.h"
#include "imu.h"

// -- Servo objects --
Servo servos[NUM_JOINTS];
static const uint8_t servo_pins[NUM_JOINTS] = {
    SERVO_PIN_J1, SERVO_PIN_J2, SERVO_PIN_J3,
    SERVO_PIN_J4, SERVO_PIN_J5
};

// Per-joint PWM limits for safety clamping
static const uint16_t pwm_min[NUM_JOINTS] = {
    PWM_MIN_J1, PWM_MIN_J2, PWM_MIN_J3, PWM_MIN_J4, PWM_MIN_J5
};
static const uint16_t pwm_max[NUM_JOINTS] = {
    PWM_MAX_J1, PWM_MAX_J2, PWM_MAX_J3, PWM_MAX_J4, PWM_MAX_J5
};

// -- State --
uint16_t target_pwm[NUM_JOINTS];
float imu_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // identity quaternion
bool imu_data_valid = false;
bool watchdog_active = false;

unsigned long last_cmd_time = 0;
unsigned long last_state_send = 0;

// -- Serial receive buffer --
uint8_t rx_buf[CMD_PACKET_SIZE];
uint8_t rx_idx = 0;
bool rx_synced = false;

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Initialize servos at neutral position
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        target_pwm[i] = PWM_NEUTRAL;
        servos[i].attach(servo_pins[i]);
        servos[i].writeMicroseconds(PWM_NEUTRAL);
    }

    // Status LED
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // Initialize IMU (no-op if IMU_ENABLED == 0)
    imu_data_valid = imu_init();
    if (imu_data_valid) {
        digitalWrite(STATUS_LED, HIGH);  // LED on when IMU is ready
    }

    last_cmd_time = millis();
    last_state_send = millis();
}

void loop() {
    // ---- 1. Read and parse incoming serial commands ----
    while (Serial.available() > 0) {
        uint8_t byte = Serial.read();

        if (!rx_synced) {
            // Scanning for sync byte
            if (byte == SYNC_CMD) {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_synced = true;
            }
            continue;
        }

        // Accumulate bytes
        rx_buf[rx_idx++] = byte;

        if (rx_idx >= CMD_PACKET_SIZE) {
            // Full packet received, try to parse
            CommandPacket cmd;
            if (parse_command(rx_buf, &cmd)) {
                // Apply commanded positions with safety clamping
                for (uint8_t i = 0; i < NUM_JOINTS; i++) {
                    uint16_t pwm = cmd.pwm[i];
                    if (pwm < pwm_min[i]) pwm = pwm_min[i];
                    if (pwm > pwm_max[i]) pwm = pwm_max[i];
                    target_pwm[i] = pwm;
                }

                // Handle flags
                if (cmd.flags & 0x01) {
                    digitalWrite(STATUS_LED, HIGH);
                } else {
                    // Only control LED via flags if IMU is not driving it
                    #if !IMU_ENABLED
                    digitalWrite(STATUS_LED, LOW);
                    #endif
                }

                last_cmd_time = millis();
                watchdog_active = false;
            }

            // Reset for next packet
            rx_synced = false;
            rx_idx = 0;
        }
    }

    // ---- 2. Watchdog: return to neutral if no commands received ----
    if (millis() - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
        if (!watchdog_active) {
            for (uint8_t i = 0; i < NUM_JOINTS; i++) {
                target_pwm[i] = PWM_NEUTRAL;
            }
            watchdog_active = true;
        }
    }

    // ---- 3. Write servo positions ----
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        servos[i].writeMicroseconds(target_pwm[i]);
    }

    // ---- 4. Read IMU (non-blocking) ----
    #if IMU_ENABLED
    if (imu_read(imu_quat)) {
        imu_data_valid = true;
    }
    #endif

    // ---- 5. Send state packet at fixed rate ----
    unsigned long now = millis();
    if (now - last_state_send >= STATE_INTERVAL_MS) {
        last_state_send = now;

        uint8_t status = 0;
        if (imu_data_valid) status |= 0x01;
        if (watchdog_active) status |= 0x02;

        #if IMU_ENABLED
        uint8_t state_buf[STATE_IMU_PACKET_SIZE];
        build_state_imu(state_buf, imu_quat, target_pwm, status);
        Serial.write(state_buf, STATE_IMU_PACKET_SIZE);
        #else
        uint8_t state_buf[STATE_NO_IMU_PACKET_SIZE];
        build_state_no_imu(state_buf, target_pwm, status);
        Serial.write(state_buf, STATE_NO_IMU_PACKET_SIZE);
        #endif
    }
}
