mod adelino_config;

use std::path::{Path, PathBuf};
use std::process::Command;

use clap::{Parser, Subcommand};
use tracing::{error, info, warn};

use arduino_controller::command_source::websocket::WebSocketSource;
use arduino_controller::controller::ArduinoController;
use arduino_controller::CalibrationData;

#[derive(Parser)]
#[command(name = "adelino-standalone")]
#[command(about = "Standalone controller for the Adelino 5-DOF servo robot")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run the controller (WebSocket server + Arduino serial bridge)
    Run {
        /// Serial port for the Arduino
        #[arg(long, default_value = "COM3")]
        port: String,

        /// WebSocket server port
        #[arg(long, default_value_t = 8765)]
        ws_port: u16,

        /// Serial baud rate
        #[arg(long, default_value_t = 115200)]
        baud: u32,

        /// Path to calibration TOML file
        #[arg(long)]
        calibration: Option<PathBuf>,
    },

    /// Flash Arduino firmware
    Flash {
        /// Serial port for the Arduino
        #[arg(long, default_value = "COM3")]
        port: String,

        /// Arduino Fully Qualified Board Name
        #[arg(long, default_value = "arduino:avr:mega:cpu=atmega2560")]
        fqbn: String,

        /// Path to firmware directory
        #[arg(long)]
        firmware_dir: Option<PathBuf>,
    },

    /// Interactive servo calibration
    Calibrate {
        /// Serial port for the Arduino
        #[arg(long, default_value = "COM3")]
        port: String,

        /// Output calibration file path
        #[arg(long, default_value = "calibration.toml")]
        output: PathBuf,
    },

    /// Test servos by sweeping through their range
    Test {
        /// Serial port for the Arduino
        #[arg(long, default_value = "COM3")]
        port: String,

        /// Test only a specific joint (1-5), or all if omitted
        #[arg(long)]
        joint: Option<u8>,

        /// Path to calibration TOML file
        #[arg(long)]
        calibration: Option<PathBuf>,
    },
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Run {
            port,
            ws_port,
            baud,
            calibration,
        } => {
            if let Err(e) = run_controller(port, ws_port, baud, calibration).await {
                error!("Controller error: {e}");
                std::process::exit(1);
            }
        }
        Commands::Flash {
            port,
            fqbn,
            firmware_dir,
        } => {
            if let Err(e) = flash_firmware(&port, &fqbn, firmware_dir) {
                error!("Flash error: {e}");
                std::process::exit(1);
            }
        }
        Commands::Calibrate { port, output } => {
            if let Err(e) = run_calibration(&port, &output).await {
                error!("Calibration error: {e}");
                std::process::exit(1);
            }
        }
        Commands::Test {
            port,
            joint,
            calibration,
        } => {
            if let Err(e) = run_test(&port, joint, calibration).await {
                error!("Test error: {e}");
                std::process::exit(1);
            }
        }
    }
}

async fn run_controller(
    port: String,
    ws_port: u16,
    baud: u32,
    calibration_path: Option<PathBuf>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut config = adelino_config::default_config();
    config.serial_baud = baud;

    let source = WebSocketSource::new(ws_port);
    let mut controller = ArduinoController::new(config, source, port);

    // Load calibration if available
    if let Some(path) = calibration_path {
        match CalibrationData::load(&path) {
            Ok(cal) => {
                info!("Loaded calibration from {}", path.display());
                controller = controller.with_calibrations(cal.joints);
            }
            Err(e) => {
                warn!("Failed to load calibration from {}: {e}", path.display());
                warn!("Using default calibration");
            }
        }
    } else {
        // Try default path
        let default_path = PathBuf::from("calibration.toml");
        if default_path.exists() {
            match CalibrationData::load(&default_path) {
                Ok(cal) => {
                    info!("Loaded calibration from calibration.toml");
                    controller = controller.with_calibrations(cal.joints);
                }
                Err(e) => {
                    warn!("Failed to load calibration.toml: {e}");
                }
            }
        }
    }

    controller.run().await?;
    Ok(())
}

fn flash_firmware(
    port: &str,
    fqbn: &str,
    firmware_dir: Option<PathBuf>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Find firmware directory
    let firmware_dir = firmware_dir.unwrap_or_else(find_firmware_dir);

    if !firmware_dir.exists() {
        return Err(format!(
            "Firmware directory not found: {}. Use --firmware-dir to specify it.",
            firmware_dir.display()
        )
        .into());
    }

    info!("Firmware directory: {}", firmware_dir.display());

    // Check arduino-cli is available
    let check = Command::new("arduino-cli").arg("version").output();
    if check.is_err() {
        return Err(
            "arduino-cli not found on PATH. Install it from https://arduino.github.io/arduino-cli/"
                .into(),
        );
    }

    // Compile
    info!("Compiling firmware...");
    let compile = Command::new("arduino-cli")
        .args(["compile", "--fqbn", fqbn])
        .arg(&firmware_dir)
        .status()?;
    if !compile.success() {
        return Err("Firmware compilation failed".into());
    }
    info!("Compilation successful");

    // Upload
    info!("Uploading to {port}...");
    let upload = Command::new("arduino-cli")
        .args(["upload", "-p", port, "--fqbn", fqbn])
        .arg(&firmware_dir)
        .status()?;
    if !upload.success() {
        return Err("Firmware upload failed".into());
    }

    info!("Firmware flashed successfully to {port}");
    Ok(())
}

fn find_firmware_dir() -> PathBuf {
    // Check env var first
    if let Ok(dir) = std::env::var("ADELINO_FIRMWARE_DIR") {
        return PathBuf::from(dir);
    }

    // Relative to executable (target/ is at workspace root, so exe is in target/release/)
    if let Ok(exe) = std::env::current_exe() {
        let candidate = exe.parent().unwrap().join("../../firmware");
        if candidate.exists() {
            return candidate;
        }
    }

    // Workspace default (running from the adelino project root)
    PathBuf::from("firmware")
}

async fn run_calibration(
    port: &str,
    output: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::io::{self, BufRead, Write};

    let config = adelino_config::default_config();
    let mut transport =
        arduino_controller::transport::SerialTransport::open(port, config.serial_baud)?;

    info!("Starting calibration for {} joints", config.joints.len());
    println!("\n=== Adelino Servo Calibration ===");
    println!("This process will calibrate each servo's center position and range.");
    println!("Make sure the robot is in a safe position.\n");

    let stdin = io::stdin();
    let mut stdout = io::stdout();
    let mut calibrations = Vec::new();

    for joint in &config.joints {
        println!("--- Calibrating {} (pin {}, {}) ---", joint.name, joint.pin, joint.servo_model);

        // Step 1: Find center
        println!("Step 1: Finding center position");
        println!("  Use +/- keys to nudge the servo. Press Enter when at mechanical center.");
        let mut current_pwm = joint.pwm_center_us;
        let center_pwm = loop {
            // Send current position
            let mut pwm = [1500u16; 5];
            pwm[config.joints.iter().position(|j| j.name == joint.name).unwrap()] = current_pwm;
            arduino_controller::transport::send_raw_pwm(&mut transport, pwm)?;

            print!("  PWM: {current_pwm}  (+/-/Enter): ");
            stdout.flush()?;

            let mut line = String::new();
            stdin.lock().read_line(&mut line)?;
            let line = line.trim();

            if line.is_empty() {
                break current_pwm;
            } else if let Some(rest) = line.strip_prefix('+') {
                let step: u16 = rest.parse().unwrap_or(10);
                current_pwm = (current_pwm + step).min(2500);
            } else if let Some(rest) = line.strip_prefix('-') {
                let step: u16 = rest.parse().unwrap_or(10);
                current_pwm = current_pwm.saturating_sub(step);
            } else if let Ok(pwm) = line.parse::<u16>() {
                current_pwm = pwm.clamp(500, 2500);
            }
        };
        println!("  Center PWM: {center_pwm}");

        // Step 2: Positive range
        println!("Step 2: Finding positive range limit");
        println!("  Nudge with +/- keys. Press Enter at the safe physical limit.");
        current_pwm = center_pwm;
        let max_pwm = loop {
            let mut pwm = [1500u16; 5];
            pwm[config.joints.iter().position(|j| j.name == joint.name).unwrap()] = current_pwm;
            arduino_controller::transport::send_raw_pwm(&mut transport, pwm)?;

            print!("  PWM: {current_pwm} (delta: +{})  (+/-/Enter): ", current_pwm - center_pwm);
            stdout.flush()?;

            let mut line = String::new();
            stdin.lock().read_line(&mut line)?;
            let line = line.trim();

            if line.is_empty() {
                break current_pwm;
            } else if let Some(rest) = line.strip_prefix('+') {
                let step: u16 = rest.parse().unwrap_or(10);
                current_pwm = (current_pwm + step).min(2500);
            } else if let Some(rest) = line.strip_prefix('-') {
                let step: u16 = rest.parse().unwrap_or(10);
                current_pwm = current_pwm.saturating_sub(step);
            }
        };

        // Step 3: Negative range
        println!("Step 3: Finding negative range limit");
        println!("  Nudge with +/- keys. Press Enter at the safe physical limit.");
        current_pwm = center_pwm;
        let min_pwm = loop {
            let mut pwm = [1500u16; 5];
            pwm[config.joints.iter().position(|j| j.name == joint.name).unwrap()] = current_pwm;
            arduino_controller::transport::send_raw_pwm(&mut transport, pwm)?;

            print!("  PWM: {current_pwm} (delta: {})  (+/-/Enter): ", current_pwm as i32 - center_pwm as i32);
            stdout.flush()?;

            let mut line = String::new();
            stdin.lock().read_line(&mut line)?;
            let line = line.trim();

            if line.is_empty() {
                break current_pwm;
            } else if let Some(rest) = line.strip_prefix('+') {
                let step: u16 = rest.parse().unwrap_or(10);
                current_pwm = (current_pwm + step).min(2500);
            } else if let Some(rest) = line.strip_prefix('-') {
                let step: u16 = rest.parse().unwrap_or(10);
                current_pwm = current_pwm.saturating_sub(step);
            }
        };

        // Compute calibration
        let pos_delta_pwm = (max_pwm as f32 - center_pwm as f32).abs();
        let neg_delta_pwm = (center_pwm as f32 - min_pwm as f32).abs();
        let max_rad = joint.angle_max_rad;
        let min_rad = joint.angle_min_rad;

        let pwm_per_rad_pos = if max_rad.abs() > 1e-6 {
            pos_delta_pwm / max_rad
        } else {
            0.0
        };
        let pwm_per_rad_neg = if min_rad.abs() > 1e-6 {
            neg_delta_pwm / min_rad.abs()
        } else {
            0.0
        };

        // Step 4: Direction check
        println!("Step 4: Direction check");
        let test_rad = 0.1;
        let test_pwm = (center_pwm as f32 + test_rad * pwm_per_rad_pos) as u16;
        let mut pwm_arr = [1500u16; 5];
        let joint_idx = config.joints.iter().position(|j| j.name == joint.name).unwrap();
        pwm_arr[joint_idx] = test_pwm;
        arduino_controller::transport::send_raw_pwm(&mut transport, pwm_arr)?;

        print!("  Moved to +0.1 rad ({test_pwm} PWM). Is this the positive direction? (y/n): ");
        stdout.flush()?;
        let mut line = String::new();
        stdin.lock().read_line(&mut line)?;
        let inverted = line.trim().to_lowercase() != "y";

        // Return to center
        pwm_arr[joint_idx] = center_pwm;
        arduino_controller::transport::send_raw_pwm(&mut transport, pwm_arr)?;

        let cal = arduino_controller::JointCalibration {
            name: joint.name.clone(),
            center_pwm,
            pwm_per_rad_pos,
            pwm_per_rad_neg,
            min_rad,
            max_rad,
            inverted,
        };

        println!("  {} calibrated: center={}, pos_scale={:.1}, neg_scale={:.1}, inverted={}",
            cal.name, cal.center_pwm, cal.pwm_per_rad_pos, cal.pwm_per_rad_neg, cal.inverted);
        println!();

        calibrations.push(cal);
    }

    // Save calibration
    let cal_data = CalibrationData {
        robot_name: config.name.clone(),
        joints: calibrations,
        created_at: chrono_now(),
        notes: String::new(),
    };
    cal_data.save(output)?;

    println!("\n=== Calibration saved to {} ===", output.display());
    println!("Use --calibration {} with the run command to load it.", output.display());

    Ok(())
}

async fn run_test(
    port: &str,
    joint_filter: Option<u8>,
    calibration_path: Option<PathBuf>,
) -> Result<(), Box<dyn std::error::Error>> {
    let config = adelino_config::default_config();
    let mut transport =
        arduino_controller::transport::SerialTransport::open(port, config.serial_baud)?;

    // Load calibrations
    let calibrations: Vec<arduino_controller::JointCalibration> = if let Some(path) = calibration_path {
        CalibrationData::load(&path)?.joints
    } else if PathBuf::from("calibration.toml").exists() {
        CalibrationData::load(&PathBuf::from("calibration.toml"))?.joints
    } else {
        config.joints.iter().map(|j| j.to_default_calibration()).collect()
    };

    let test_joints: Vec<usize> = match joint_filter {
        Some(j) => vec![(j as usize) - 1],
        None => (0..config.joints.len()).collect(),
    };

    println!("=== Servo Test ===");
    for &idx in &test_joints {
        let cal = &calibrations[idx];
        println!("Testing {} (center={}, range=[{:.2}, {:.2}] rad)",
            cal.name, cal.center_pwm, cal.min_rad, cal.max_rad);

        // Sweep: center -> max -> center -> min -> center
        let steps = [0.0, cal.max_rad, 0.0, cal.min_rad, 0.0];
        for &target_rad in &steps {
            let target_pwm = cal.rad_to_pwm(target_rad);
            let mut pwm = [1500u16; 5];
            for (i, c) in calibrations.iter().enumerate() {
                pwm[i] = c.center_pwm; // others at center
            }
            pwm[idx] = target_pwm;
            println!("  -> {:.2} rad ({} PWM)", target_rad, target_pwm);
            arduino_controller::transport::send_raw_pwm(&mut transport, pwm)?;
            tokio::time::sleep(std::time::Duration::from_millis(800)).await;
        }
        println!();
    }

    println!("Test complete.");
    Ok(())
}

fn chrono_now() -> String {
    // Simple ISO 8601 timestamp without chrono dependency
    let duration = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    format!("unix:{}", duration.as_secs())
}
