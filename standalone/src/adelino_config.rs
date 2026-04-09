use std::f32::consts::FRAC_PI_2;

use arduino_controller::config::{JointConfig, RobotConfig};

/// Default Adelino configuration based on embodiment specs.
/// These values are used when no calibration file is present.
pub fn default_config() -> RobotConfig {
    RobotConfig {
        name: "adelino".into(),
        joints: vec![
            JointConfig {
                name: "J1".into(),
                servo_model: "HK15338".into(),
                pin: 2,
                pwm_min_us: 500,
                pwm_max_us: 2500,
                pwm_center_us: 1500,
                angle_min_rad: -FRAC_PI_2,
                angle_max_rad: FRAC_PI_2,
                inverted: false,
            },
            JointConfig {
                name: "J2".into(),
                servo_model: "HK15298B".into(),
                pin: 3,
                pwm_min_us: 500,
                pwm_max_us: 2500,
                pwm_center_us: 1500,
                angle_min_rad: -FRAC_PI_2,
                angle_max_rad: FRAC_PI_2,
                inverted: false,
            },
            JointConfig {
                name: "J3".into(),
                servo_model: "HK15328A".into(),
                pin: 4,
                pwm_min_us: 500,
                pwm_max_us: 2500,
                pwm_center_us: 1500,
                angle_min_rad: -FRAC_PI_2,
                angle_max_rad: FRAC_PI_2,
                inverted: false,
            },
            JointConfig {
                name: "J4".into(),
                servo_model: "HK15138".into(),
                pin: 5,
                pwm_min_us: 500,
                pwm_max_us: 2500,
                pwm_center_us: 1500,
                angle_min_rad: -FRAC_PI_2,
                angle_max_rad: FRAC_PI_2,
                inverted: false,
            },
            JointConfig {
                name: "J5".into(),
                servo_model: "HK15178".into(),
                pin: 6,
                pwm_min_us: 500,
                pwm_max_us: 2500,
                pwm_center_us: 1500,
                angle_min_rad: -FRAC_PI_2,
                angle_max_rad: FRAC_PI_2,
                inverted: false,
            },
        ],
        serial_baud: 115200,
        command_timeout_ms: 500,
        control_rate_hz: 50,
    }
}
