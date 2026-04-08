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
#include "led_matrix.h"

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
bool watchdog_disabled = false;

unsigned long last_cmd_time = 0;
unsigned long last_state_send = 0;

// -- Serial receive buffer --
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_idx = 0;
uint8_t rx_expected = 0;  // expected packet size (set after reading type byte)
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

    // Initialize LED matrix (no-op if LED_MATRIX_ENABLED == 0)
    led_matrix_init();

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
            if (byte == SYNC_CMD) {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_expected = 0;  // need type byte next
                rx_synced = true;
            }
            continue;
        }

        rx_buf[rx_idx++] = byte;

        // After receiving the type byte, determine expected packet size
        if (rx_idx == 2) {
            rx_expected = expected_cmd_size(byte);
            if (rx_expected == 0) {
                // Unknown type — resync
                rx_synced = false;
                rx_idx = 0;
                continue;
            }
        }

        if (rx_expected > 0 && rx_idx >= rx_expected) {
            // Full packet received — dispatch by type
            switch (rx_buf[1]) {
                case TYPE_CMD: {
                    CommandPacket cmd;
                    if (parse_command(rx_buf, &cmd)) {
                        for (uint8_t i = 0; i < NUM_JOINTS; i++) {
                            uint16_t pwm = cmd.pwm[i];
                            if (pwm < pwm_min[i]) pwm = pwm_min[i];
                            if (pwm > pwm_max[i]) pwm = pwm_max[i];
                            target_pwm[i] = pwm;
                        }

                        if (cmd.flags & 0x01) {
                            digitalWrite(STATUS_LED, HIGH);
                        } else {
                            #if !IMU_ENABLED
                            digitalWrite(STATUS_LED, LOW);
                            #endif
                        }

                        watchdog_disabled = (cmd.flags & 0x02) != 0;
                        last_cmd_time = millis();
                        watchdog_active = false;
                    }
                    break;
                }
                case TYPE_CMD_LED: {
                    LedCommandPacket led_cmd;
                    if (parse_led_command(rx_buf, &led_cmd)) {
                        led_matrix_update(led_cmd.rgb, led_cmd.brightness);
                    }
                    break;
                }
            }

            rx_synced = false;
            rx_idx = 0;
        }
    }

    // ---- 2. Watchdog: return to neutral if no commands received ----
    if (!watchdog_disabled && millis() - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
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
