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

        /// Arduino FQBN (auto-detected from port if omitted)
        #[arg(long)]
        fqbn: Option<String>,

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

    /// Back up the current Arduino firmware to a hex file
    Backup {
        /// Serial port for the Arduino
        #[arg(long, default_value = "COM3")]
        port: String,

        /// Arduino FQBN (auto-detected from port if omitted)
        #[arg(long)]
        fqbn: Option<String>,

        /// Output hex file path
        #[arg(long, default_value = "firmware_backup.hex")]
        output: PathBuf,
    },

    /// Test LED matrix by displaying color patterns
    LedTest {
        /// Serial port for the Arduino
        #[arg(long, default_value = "COM3")]
        port: String,

        /// LED data pin on the Arduino (must match config.h LED_MATRIX_PIN)
        #[arg(long, default_value_t = 7)]
        pin: u8,
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
            let fqbn = match resolve_fqbn(fqbn, &port) {
                Ok(f) => f,
                Err(e) => {
                    error!("Board detection error: {e}");
                    std::process::exit(1);
                }
            };
            if let Err(e) = flash_firmware(&port, &fqbn, firmware_dir) {
                error!("Flash error: {e}");
                std::process::exit(1);
            }
        }
        Commands::Backup {
            port,
            fqbn,
            output,
        } => {
            let fqbn = match resolve_fqbn(fqbn, &port) {
                Ok(f) => f,
                Err(e) => {
                    error!("Board detection error: {e}");
                    std::process::exit(1);
                }
            };
            if let Err(e) = backup_firmware(&port, &fqbn, &output) {
                error!("Backup error: {e}");
                std::process::exit(1);
            }
        }
        Commands::Calibrate { port, output } => {
            if let Err(e) = run_calibration(&port, &output).await {
                error!("Calibration error: {e}");
                std::process::exit(1);
            }
        }
        Commands::LedTest { port, pin } => {
            if let Err(e) = run_led_test(&port, pin).await {
                error!("LED test error: {e}");
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

/// Use the explicit FQBN if provided, otherwise auto-detect from the port.
fn resolve_fqbn(
    explicit: Option<String>,
    port: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    if let Some(fqbn) = explicit {
        return Ok(fqbn);
    }
    info!("No --fqbn specified, detecting board on {port}...");
    let fqbn = detect_fqbn(port)?;
    info!("Detected board: {fqbn}");
    Ok(fqbn)
}

/// Run `arduino-cli board list --format json` and extract the FQBN for the given port.
///
/// The JSON structure is:
/// ```json
/// { "detected_ports": [ { "matching_boards": [{ "fqbn": "..." }],
///                          "port": { "address": "COM3" } } ] }
/// ```
fn detect_fqbn(port: &str) -> Result<String, Box<dyn std::error::Error>> {
    let output = Command::new("arduino-cli")
        .args(["board", "list", "--format", "json"])
        .output()
        .map_err(|_| "arduino-cli not found on PATH")?;

    if !output.status.success() {
        return Err("arduino-cli board list failed".into());
    }

    let text = String::from_utf8(output.stdout)?;
    let port_upper = port.to_uppercase();

    // Split on each detected port entry boundary.
    // Each block between "matching_boards" entries represents one detected port.
    for entry in text.split("\"matching_boards\"") {
        // The port address appears after this block's matching_boards section.
        // Check if this entry's port.address matches ours.
        let entry_upper = entry.to_uppercase();
        let has_port = entry_upper
            .find("\"ADDRESS\"")
            .and_then(|pos| {
                let after = &entry_upper[pos..];
                after.find(&port_upper)
            })
            .is_some();

        if !has_port {
            continue;
        }

        // Extract fqbn from this entry's matching_boards
        if let Some(result) = extract_json_string_value(entry, "fqbn") {
            if !result.is_empty() {
                return Ok(result);
            }
        }
    }

    Err(format!(
        "Could not detect board on {port}. Is it plugged in? \
         You can specify the board manually with --fqbn (run `arduino-cli board list` to check)."
    )
    .into())
}

/// Extract the first string value for a given key from a JSON fragment.
/// e.g. for key "fqbn", finds `"fqbn": "arduino:avr:uno"` and returns `arduino:avr:uno`.
fn extract_json_string_value(text: &str, key: &str) -> Option<String> {
    let pattern = format!("\"{key}\"");
    let key_pos = text.find(&pattern)?;
    let after_key = &text[key_pos + pattern.len()..];
    // Skip whitespace and colon
    let after_colon = after_key.find(':').map(|p| &after_key[p + 1..])?;
    let q_start = after_colon.find('"')?;
    let after_q = &after_colon[q_start + 1..];
    let q_end = after_q.find('"')?;
    Some(after_q[..q_end].to_string())
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

fn backup_firmware(
    port: &str,
    fqbn: &str,
    output: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    // Extract MCU and programmer from FQBN
    // e.g. "arduino:avr:mega:cpu=atmega2560" -> "atmega2560", programmer "wiring"
    let (mcu, programmer, baud) = parse_fqbn_for_avrdude(fqbn)?;

    // Locate avrdude via arduino-cli
    let avrdude = find_avrdude()?;
    info!("Using avrdude: {}", avrdude.display());

    info!("Reading firmware from {port} into {}...", output.display());
    let flash_arg = format!("flash:r:{}:i", output.display());
    let status = Command::new(&avrdude)
        .args(["-p", &mcu, "-c", &programmer, "-P", port, "-b", &baud])
        .args(["-U", &flash_arg])
        .status()?;

    if !status.success() {
        return Err("avrdude failed to read firmware".into());
    }

    info!("Firmware backed up to {}", output.display());
    info!("Restore later with: adelino-standalone flash --port {port}");
    Ok(())
}

/// Extract MCU, programmer, and baud from an Arduino FQBN.
fn parse_fqbn_for_avrdude(fqbn: &str) -> Result<(String, String, String), Box<dyn std::error::Error>> {
    // FQBN format: "vendor:arch:board:options"
    // Currently only supports arduino:avr:mega:cpu=atmega2560
    let parts: Vec<&str> = fqbn.split(':').collect();
    if parts.len() < 3 {
        return Err(format!("Invalid FQBN: {fqbn}").into());
    }

    let board = parts[2];
    match board {
        "mega" => {
            // Extract CPU from options if present, default to atmega2560
            let mcu = parts
                .get(3)
                .and_then(|opts| opts.strip_prefix("cpu="))
                .unwrap_or("atmega2560")
                .to_string();
            Ok((mcu, "wiring".to_string(), "115200".to_string()))
        }
        "uno" => Ok(("atmega328p".to_string(), "arduino".to_string(), "115200".to_string())),
        "nano" => Ok(("atmega328p".to_string(), "arduino".to_string(), "57600".to_string())),
        "leonardo" => Ok(("atmega32u4".to_string(), "avr109".to_string(), "57600".to_string())),
        _ => Err(format!(
            "Unsupported board '{board}' in FQBN. Supported: mega, uno, nano, leonardo"
        )
        .into()),
    }
}

/// Find avrdude, preferring the one bundled with arduino-cli.
fn find_avrdude() -> Result<PathBuf, Box<dyn std::error::Error>> {
    let avrdude_rel = Path::new("packages/arduino/tools/avrdude");

    // Collect candidate Arduino data directories
    let mut data_dirs: Vec<PathBuf> = Vec::new();

    // 1. Try arduino-cli config
    if let Ok(output) = Command::new("arduino-cli")
        .args(["config", "dump", "--format", "json"])
        .output()
    {
        if output.status.success() {
            if let Ok(text) = String::from_utf8(output.stdout) {
                if let Some(dir) = extract_arduino_data_dir(&text) {
                    data_dirs.push(PathBuf::from(dir));
                }
            }
        }
    }

    // 2. Platform-specific defaults
    if cfg!(windows) {
        if let Ok(local) = std::env::var("LOCALAPPDATA") {
            data_dirs.push(PathBuf::from(local).join("Arduino15"));
        }
    } else if cfg!(target_os = "macos") {
        if let Ok(home) = std::env::var("HOME") {
            data_dirs.push(PathBuf::from(home).join("Library/Arduino15"));
        }
    } else {
        if let Ok(home) = std::env::var("HOME") {
            data_dirs.push(PathBuf::from(home).join(".arduino15"));
        }
    }

    // Search each data directory for the latest avrdude version
    for data_dir in &data_dirs {
        let avrdude_base = data_dir.join(avrdude_rel);
        if let Ok(entries) = std::fs::read_dir(&avrdude_base) {
            let mut versions: Vec<PathBuf> = entries
                .filter_map(|e| e.ok())
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect();
            versions.sort();
            if let Some(version_dir) = versions.last() {
                let avrdude_exe = version_dir.join("bin/avrdude.exe");
                if avrdude_exe.exists() {
                    return Ok(avrdude_exe);
                }
                let avrdude = version_dir.join("bin/avrdude");
                if avrdude.exists() {
                    return Ok(avrdude);
                }
            }
        }
    }

    // 3. Fallback: check if avrdude is on PATH
    if Command::new("avrdude").arg("-?").output().is_ok() {
        return Ok(PathBuf::from("avrdude"));
    }

    Err(
        "avrdude not found. Install arduino-cli (which bundles avrdude) \
         or install avrdude separately and ensure it is on PATH."
            .into(),
    )
}

/// Extract the data directory from arduino-cli JSON config output.
fn extract_arduino_data_dir(json_text: &str) -> Option<String> {
    let dirs_start = json_text.find("\"directories\"")?;
    let after_dirs = &json_text[dirs_start..];
    let data_start = after_dirs.find("\"data\"")?;
    let after_data = &after_dirs[data_start..];
    let colon = after_data.find(':')?;
    let after_colon = &after_data[colon + 1..];
    let quote_start = after_colon.find('"')?;
    let after_quote = &after_colon[quote_start + 1..];
    let quote_end = after_quote.find('"')?;
    Some(after_quote[..quote_end].replace("\\\\", "/").replace('\\', "/"))
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

/// Send a PWM command with the watchdog disabled (flag 0x02),
/// then drain any pending state packets so the serial receive buffer
/// doesn't overflow (which causes "semaphore timeout" on Windows).
fn send_cal_cmd(
    transport: &mut arduino_controller::transport::SerialTransport,
    pwm: [u16; 5],
) -> std::io::Result<()> {
    use arduino_controller::transport::protocol::CommandPacket;
    // Drain before sending — the buffer may have filled while waiting for input
    while transport.try_read_state()?.is_some() {}
    transport.send_command(&CommandPacket { pwm, flags: 0x02 })?;
    while transport.try_read_state()?.is_some() {}
    Ok(())
}

/// Wait for a key event, draining the serial buffer every 100ms so
/// the Arduino's state packets don't pile up and trigger os error 121.
fn wait_key(
    transport: &mut arduino_controller::transport::SerialTransport,
) -> Result<crossterm::event::KeyEvent, Box<dyn std::error::Error>> {
    use crossterm::event::{self, Event, KeyEventKind};
    use std::time::Duration;
    loop {
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(ke) = event::read()? {
                // Only react to key-press events; ignore release/repeat
                // so that e.g. releasing Enter doesn't auto-confirm the next step.
                if ke.kind == KeyEventKind::Press {
                    return Ok(ke);
                }
            }
        }
        // No key yet — drain serial buffer while idle
        while transport.try_read_state()?.is_some() {}
    }
}

/// Smoothly move a single joint from `from_pwm` to `to_pwm` over `duration`,
/// updating the display with the current value. Uses 50Hz steps.
fn smooth_move(
    transport: &mut arduino_controller::transport::SerialTransport,
    make_pwm: &dyn Fn(u16) -> [u16; 5],
    from_pwm: u16,
    to_pwm: u16,
    center_pwm: u16,
    duration: std::time::Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::io::Write;
    let steps = (duration.as_millis() as f32 / 20.0).max(1.0) as u32; // 50Hz
    let step_duration = duration / steps;
    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        let pwm = from_pwm as f32 + t * (to_pwm as f32 - from_pwm as f32);
        let pwm = (pwm.round() as u16).clamp(500, 2500);
        send_cal_cmd(transport, make_pwm(pwm))?;
        print!("\r  PWM: {pwm} (delta: {:+})  ", pwm as i32 - center_pwm as i32);
        std::io::stdout().flush()?;
        std::thread::sleep(step_duration);
    }
    Ok(())
}

async fn run_calibration(
    port: &str,
    output: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::io::Write;
    use crossterm::event::{KeyCode, KeyModifiers};
    use crossterm::terminal;

    let config = adelino_config::default_config();
    let mut transport =
        arduino_controller::transport::SerialTransport::open(port, config.serial_baud)?;

    // Load existing calibration if the output file already exists
    let existing_cal = if output.exists() {
        match CalibrationData::load(output) {
            Ok(cal) => {
                info!("Loaded existing calibration from {}", output.display());
                Some(cal)
            }
            Err(e) => {
                warn!("Failed to load existing calibration: {e}, starting from defaults");
                None
            }
        }
    } else {
        None
    };

    info!("Starting calibration for {} joints", config.joints.len());
    println!("\n=== Adelino Servo Calibration ===");
    println!("This process will calibrate each servo's center position and range.");
    println!("Make sure the robot is in a safe position.");
    println!("Watchdog is disabled during calibration -- servos hold position.");
    if existing_cal.is_some() {
        println!("Preloaded existing calibration from {}", output.display());
    }
    println!();

    let mut stdout = std::io::stdout();
    let mut calibrations: Vec<arduino_controller::JointCalibration> = Vec::new();

    // Enable raw mode so keypresses are handled immediately
    terminal::enable_raw_mode()?;

    // Drain any stale terminal events (e.g. the Enter from launching the command)
    {
        use crossterm::event;
        use std::time::Duration;
        while event::poll(Duration::from_millis(50))? {
            let _ = event::read()?;
        }
    }

    // Ensure raw mode is disabled on early return / panic
    let result = (|| -> Result<Vec<arduino_controller::JointCalibration>, Box<dyn std::error::Error>> {
        for joint in &config.joints {
            let joint_idx = config.joints.iter().position(|j| j.name == joint.name).unwrap();

            print!("--- Calibrating {} (pin {}, {}) ---\r\n", joint.name, joint.pin, joint.servo_model);

            // Build a pwm array: already-calibrated joints hold their center,
            // not-yet-calibrated joints use existing calibration or config default.
            let make_pwm = |val: u16| -> [u16; 5] {
                let mut pwm = [1500u16; 5];
                for (i, jc) in config.joints.iter().enumerate() {
                    if i == joint_idx {
                        pwm[i] = val;
                    } else if let Some(done) = calibrations.iter().find(|c| c.name == jc.name) {
                        pwm[i] = done.center_pwm;
                    } else if let Some(ec) = existing_cal.as_ref().and_then(|c| c.joints.iter().find(|j| j.name == jc.name)) {
                        pwm[i] = ec.center_pwm;
                    } else {
                        pwm[i] = jc.pwm_center_us;
                    }
                }
                pwm
            };

            // Use existing calibration's center if available, else config default
            let prev_cal = existing_cal.as_ref().and_then(|c| {
                c.joints.iter().find(|j| j.name == joint.name)
            });

            // --- Step 1: Find center ---
            print!("Step 1: Finding center position\r\n");
            print!("  +/- = nudge 10us | Enter = confirm\r\n");
            let mut current_pwm = prev_cal.map_or(joint.pwm_center_us, |c| c.center_pwm);
            send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
            print!("  PWM: {current_pwm}  ");
            stdout.flush()?;
            let center_pwm = loop {
                let ke = wait_key(&mut transport)?;
                if ke.modifiers.contains(KeyModifiers::CONTROL) && ke.code == KeyCode::Char('c') {
                    return Err("Calibration cancelled by user".into());
                }
                match ke.code {
                    KeyCode::Char('+') | KeyCode::Char('=') => {
                        current_pwm = (current_pwm + 10).min(2500);
                        send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
                        print!("\r  PWM: {current_pwm}  ");
                        stdout.flush()?;
                    }
                    KeyCode::Char('-') => {
                        current_pwm = current_pwm.saturating_sub(10);
                        send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
                        print!("\r  PWM: {current_pwm}  ");
                        stdout.flush()?;
                    }
                    KeyCode::Enter => {
                        print!("\r\n");
                        break current_pwm;
                    }
                    _ => {}
                }
            };
            print!("  Center PWM: {center_pwm}\r\n");

            // --- Step 2: Positive range ---
            print!("Step 2: Finding positive range limit\r\n");
            print!("  +/- = nudge 10us | Enter = confirm\r\n");
            current_pwm = center_pwm;
            // If we have a previous calibration, smoothly move to the old max
            if let Some(pc) = prev_cal {
                let prev_max = (center_pwm as f32 + pc.pwm_per_rad_pos * pc.max_rad).round() as u16;
                let prev_max = prev_max.clamp(500, 2500);
                print!("  Moving to previous max ({prev_max})...\r\n");
                smooth_move(&mut transport, &make_pwm, center_pwm, prev_max, center_pwm, std::time::Duration::from_secs(1))?;
                current_pwm = prev_max;
                print!("\r\n");
            } else {
                send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
            }
            print!("\r  PWM: {current_pwm} (delta: {:+})  ", current_pwm as i32 - center_pwm as i32);
            stdout.flush()?;
            let max_pwm = loop {
                let ke = wait_key(&mut transport)?;
                if ke.modifiers.contains(KeyModifiers::CONTROL) && ke.code == KeyCode::Char('c') {
                    return Err("Calibration cancelled by user".into());
                }
                match ke.code {
                    KeyCode::Char('+') | KeyCode::Char('=') => {
                        current_pwm = (current_pwm + 10).min(2500);
                        send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
                        print!("\r  PWM: {current_pwm} (delta: {:+})  ", current_pwm as i32 - center_pwm as i32);
                        stdout.flush()?;
                    }
                    KeyCode::Char('-') => {
                        current_pwm = current_pwm.saturating_sub(10);
                        send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
                        print!("\r  PWM: {current_pwm} (delta: {:+})  ", current_pwm as i32 - center_pwm as i32);
                        stdout.flush()?;
                    }
                    KeyCode::Enter => {
                        print!("\r\n");
                        break current_pwm;
                    }
                    _ => {}
                }
            };

            // --- Step 3: Negative range ---
            print!("Step 3: Finding negative range limit\r\n");
            print!("  +/- = nudge 10us | Enter = confirm\r\n");
            // First return to center from wherever step 2 left us
            if current_pwm != center_pwm {
                smooth_move(&mut transport, &make_pwm, current_pwm, center_pwm, center_pwm, std::time::Duration::from_secs(1))?;
                print!("\r\n");
            }
            current_pwm = center_pwm;
            // If we have a previous calibration, smoothly move to the old min
            if let Some(pc) = prev_cal {
                let prev_min = (center_pwm as f32 - pc.pwm_per_rad_neg * pc.min_rad.abs()).round() as u16;
                let prev_min = prev_min.clamp(500, 2500);
                print!("  Moving to previous min ({prev_min})...\r\n");
                smooth_move(&mut transport, &make_pwm, center_pwm, prev_min, center_pwm, std::time::Duration::from_secs(1))?;
                current_pwm = prev_min;
                print!("\r\n");
            } else {
                send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
            }
            print!("\r  PWM: {current_pwm} (delta: {:+})  ", current_pwm as i32 - center_pwm as i32);
            stdout.flush()?;
            let min_pwm = loop {
                let ke = wait_key(&mut transport)?;
                if ke.modifiers.contains(KeyModifiers::CONTROL) && ke.code == KeyCode::Char('c') {
                    return Err("Calibration cancelled by user".into());
                }
                match ke.code {
                    KeyCode::Char('+') | KeyCode::Char('=') => {
                        current_pwm = (current_pwm + 10).min(2500);
                        send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
                        print!("\r  PWM: {current_pwm} (delta: {:+})  ", current_pwm as i32 - center_pwm as i32);
                        stdout.flush()?;
                    }
                    KeyCode::Char('-') => {
                        current_pwm = current_pwm.saturating_sub(10);
                        send_cal_cmd(&mut transport, make_pwm(current_pwm))?;
                        print!("\r  PWM: {current_pwm} (delta: {:+})  ", current_pwm as i32 - center_pwm as i32);
                        stdout.flush()?;
                    }
                    KeyCode::Enter => {
                        print!("\r\n");
                        break current_pwm;
                    }
                    _ => {}
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

            // --- Step 4: Direction check ---
            print!("Step 4: Direction check\r\n");
            let test_rad = 0.5;
            let test_pwm = (center_pwm as f32 + test_rad * pwm_per_rad_pos) as u16;
            // Return to center first, then smoothly move to test position
            send_cal_cmd(&mut transport, make_pwm(center_pwm))?;
            smooth_move(&mut transport, &make_pwm, center_pwm, test_pwm, center_pwm, std::time::Duration::from_secs(1))?;
            print!("\r\n");

            print!("  Moved to +0.5 rad ({test_pwm} PWM). Is this the positive direction? (y/n): ");
            stdout.flush()?;
            let inverted = loop {
                let ke = wait_key(&mut transport)?;
                if ke.modifiers.contains(KeyModifiers::CONTROL) && ke.code == KeyCode::Char('c') {
                    return Err("Calibration cancelled by user".into());
                }
                match ke.code {
                    KeyCode::Char('y') | KeyCode::Char('Y') => {
                        print!("y\r\n");
                        break false;
                    }
                    KeyCode::Char('n') | KeyCode::Char('N') => {
                        print!("n\r\n");
                        break true;
                    }
                    _ => {}
                }
            };

            // Return to center
            smooth_move(&mut transport, &make_pwm, test_pwm, center_pwm, center_pwm, std::time::Duration::from_secs(1))?;
            print!("\r\n");

            let cal = arduino_controller::JointCalibration {
                name: joint.name.clone(),
                center_pwm,
                pwm_per_rad_pos,
                pwm_per_rad_neg,
                min_rad,
                max_rad,
                inverted,
            };

            print!("  {} calibrated: center={}, pos_scale={:.1}, neg_scale={:.1}, inverted={}\r\n\r\n",
                cal.name, cal.center_pwm, cal.pwm_per_rad_pos, cal.pwm_per_rad_neg, cal.inverted);

            calibrations.push(cal);
        }
        Ok(calibrations)
    })();

    // Always restore terminal mode
    terminal::disable_raw_mode()?;

    let calibrations = result?;

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

async fn run_led_test(
    port: &str,
    pin: u8,
) -> Result<(), Box<dyn std::error::Error>> {
    use arduino_controller::transport::protocol::{LedCommandPacket, LED_COUNT};

    let config = adelino_config::default_config();
    let mut transport =
        arduino_controller::transport::SerialTransport::open(port, config.serial_baud)?;

    // Drain any buffered state packets
    while transport.try_read_state()?.is_some() {}

    println!("=== LED Matrix Test (pin {pin}) ===");
    println!();

    // Step 1: Send test pattern via protocol (tests firmware + hardware)
    println!("Step 1: Sending R/G/B on first 3 LEDs via serial protocol...");
    let mut cmd = LedCommandPacket {
        brightness: 30,
        pixels: [[0; 3]; LED_COUNT],
    };
    cmd.pixels[0] = [255, 0, 0]; // red
    cmd.pixels[1] = [0, 255, 0]; // green
    cmd.pixels[2] = [0, 0, 255]; // blue
    transport.send_led_command(&cmd)?;
    while transport.try_read_state()?.is_some() {}

    println!("  Sent. Do you see 3 LEDs (red, green, blue) on the top-left?");
    println!("  Press Enter to continue to next pattern, or 'q' to quit.");

    wait_for_key()?;

    // Step 2: Fill all 32 LEDs white at low brightness
    println!("Step 2: All 32 LEDs white (low brightness)...");
    let cmd = LedCommandPacket {
        brightness: 15,
        pixels: [[255, 255, 255]; LED_COUNT],
    };
    transport.send_led_command(&cmd)?;
    while transport.try_read_state()?.is_some() {}

    println!("  Sent. All LEDs should be dim white.");
    println!("  Press Enter to continue, or 'q' to quit.");

    wait_for_key()?;

    // Step 3: Row-by-row colors
    println!("Step 3: Row colors (red, green, blue, white)...");
    let mut cmd = LedCommandPacket {
        brightness: 30,
        pixels: [[0; 3]; LED_COUNT],
    };
    for col in 0..8 {
        cmd.pixels[0 * 8 + col] = [255, 0, 0];   // row 0: red
        cmd.pixels[1 * 8 + col] = [0, 255, 0];   // row 1: green
        cmd.pixels[2 * 8 + col] = [0, 0, 255];   // row 2: blue
        cmd.pixels[3 * 8 + col] = [255, 255, 255]; // row 3: white
    }
    transport.send_led_command(&cmd)?;
    while transport.try_read_state()?.is_some() {}

    println!("  Sent. Rows should be: red, green, blue, white (top to bottom).");
    println!("  Press Enter to clear and exit, or 'q' to quit without clearing.");

    wait_for_key()?;

    // Clear
    let cmd = LedCommandPacket {
        brightness: 0,
        pixels: [[0; 3]; LED_COUNT],
    };
    transport.send_led_command(&cmd)?;

    println!("LEDs cleared. Test complete.");
    Ok(())
}

fn wait_for_key() -> Result<(), Box<dyn std::error::Error>> {
    use std::io::{self, Read};
    let mut buf = [0u8; 1];
    loop {
        io::stdin().read_exact(&mut buf)?;
        if buf[0] == b'q' || buf[0] == b'Q' {
            return Err("Quit by user".into());
        }
        if buf[0] == b'\n' || buf[0] == b'\r' {
            return Ok(());
        }
    }
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
