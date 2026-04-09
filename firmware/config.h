#ifndef ADELINO_CONFIG_H
#define ADELINO_CONFIG_H

// ============================================================
// Adelino Robot - Arduino Uno Configuration
// ============================================================

// -- Servo Configuration --
#define NUM_JOINTS       5
#define SERVO_PIN_J1     2    // Base Yaw (HK15338)
#define SERVO_PIN_J2     3    // Pitch (HK15298B)
#define SERVO_PIN_J3     4    // Pitch (HK15328A)
#define SERVO_PIN_J4     5    // Roll (HK15138)
#define SERVO_PIN_J5     6    // Head Yaw (HK15178)

// Per-joint PWM safety limits (microseconds)
// 500-2500 allows the full ~180° range of most hobby servos.
// Actual safe limits per joint are determined during calibration.
#define PWM_MIN_J1       500
#define PWM_MAX_J1       2500
#define PWM_MIN_J2       500
#define PWM_MAX_J2       2500
#define PWM_MIN_J3       500
#define PWM_MAX_J3       2500
#define PWM_MIN_J4       500
#define PWM_MAX_J4       2500
#define PWM_MIN_J5       500
#define PWM_MAX_J5       2500

#define PWM_NEUTRAL      1500

// -- Serial Communication --
#define SERIAL_BAUD      115200

// -- Timing --
#define CONTROL_RATE_HZ  50
#define STATE_INTERVAL_MS (1000 / CONTROL_RATE_HZ)
#define WATCHDOG_TIMEOUT_MS 500

// -- Status LED --
#define STATUS_LED       13

// -- LED Matrix (Keyes 2812 8x4, WS2812B) --
// Set to 1 to compile with LED matrix support
// Set to 0 to compile without (saves ~1 KB flash)
#define LED_MATRIX_ENABLED 1
#define LED_MATRIX_PIN     7
#define LED_MATRIX_COLS    8
#define LED_MATRIX_ROWS    4
#define LED_MATRIX_COUNT   (LED_MATRIX_COLS * LED_MATRIX_ROWS)  // 32

// -- IMU (optional) --
// Set to 1 to compile with BNO085 IMU support on I2C
// Set to 0 to compile without IMU (state packets use type 0x02)
#define IMU_ENABLED      0
#define IMU_SDA          A4
#define IMU_SCL          A5

#endif // ADELINO_CONFIG_H
