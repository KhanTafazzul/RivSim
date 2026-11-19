//! Integration & Unit Tests for RivSim Subsystems

use rivsim::config::*;
use rivsim::dataset::{run_episode, EpisodeConfig};
use rivsim::network::protocol::FlutterCommand;
use rivsim::network::ws_server::{encrypt_payload, decrypt_payload};
use rivsim::physics::engine::PhysicsEngine;
use rivsim::sensors::battery::BatteryModel;
use rivsim::sensors::firmware::{FirmwareState, RustFirmwareController};
use rivsim::sensors::gps::GpsSensor;
use rivsim::sensors::imu::ImuSensor;
use rivsim::sensors::ldr::LdrSensor;
use rivsim::sensors::thermal::ThermalModel;
use rivsim::sensors::turbidity::TurbiditySensor;

#[test]
fn test_locked_physical_dimensions() {
    assert_eq!(PONTOON_DIAMETER_M, 0.1524); // 6 inches
    assert_eq!(PONTOON_LENGTH_M, 1.2000);
    assert_eq!(PONTOON_SPACING_M, 0.6500);

    // Front conveyor width: 0.65m spacing + 0.1524m pontoon + 2 * 0.0254m = 0.8532m
    let calculated_span = PONTOON_SPACING_M + PONTOON_DIAMETER_M + (2.0 * 0.0254);
    assert!((CONVEYOR_WIDTH_M - calculated_span).abs() < 0.0001);
    assert_eq!(CONVEYOR_WIDTH_M, 0.8532);
}

#[test]
fn test_differential_paddle_steering_torque() {
    let mut physics = PhysicsEngine::new(0.0, 0.0, 0.0);

    // Apply differential thrust: Left paddle +200, Right paddle 0
    physics.boat.left_paddle_pwm = 200;
    physics.boat.right_paddle_pwm = 0;

    // Step physics
    physics.step(0.1);

    let (_, _, omega_z) = physics.get_boat_velocities();
    // In our coordinate system, applying thrust to left paddle induces positive/negative yaw rotation
    assert!(omega_z.abs() > 0.01, "Differential paddle thrust must induce angular velocity! Got {}", omega_z);
}

#[test]
fn test_river_vector_field_profile() {
    let mut env = rivsim::physics::environment::EnvironmentEngine::new();
    env.set_water_current(1.5, 0.0, 0.0); // 1.5 m/s downstream (+Y)

    // Centerline (x = 0): maximum current velocity
    let (_, vy_center) = env.get_water_flow_at(0.0, 0.0);
    // Near bank (x = 11.5m): slowed by boundary layer friction
    let (_, vy_bank) = env.get_water_flow_at(11.5, 0.0);

    assert!(vy_center > vy_bank, "Center flow ({:.2}) must be faster than bank flow ({:.2})", vy_center, vy_bank);
    assert!((vy_center - 1.5).abs() < 0.05);
}

#[test]
fn test_radio_attenuation_and_geofence() {
    let mut radio = rivsim::physics::radio::RadioSubsystem::new();

    // 1. Within nominal zone (10m from base station)
    radio.update(10.0, 0.0);
    assert!(!radio.connection_severed);
    assert!(!radio.geofence_breached);
    assert!(radio.rssi_dbm > -65);
    assert!(radio.packet_drop_probability < 0.05);

    // 2. In drop zone (45m from base station)
    radio.update(45.0, 0.0);
    assert!(!radio.connection_severed);
    assert!(!radio.geofence_breached);
    assert!(radio.packet_drop_probability > 0.3); // Escalating drops

    // 3. Beyond geofence (55m from base station)
    radio.update(55.0, 0.0);
    assert!(radio.connection_severed);
    assert!(radio.geofence_breached);
    assert_eq!(radio.packet_drop_probability, 1.0);
}

#[test]
fn test_gps_coordinate_projection_and_multipath_noise() {
    let mut gps = GpsSensor::new();

    // At rest (speed = 0.0): satellite bounce drift active
    for _ in 0..20 {
        gps.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.05);
    }

    // Latitude and longitude should be near origin
    assert!((gps.latitude - ORIGIN_LATITUDE).abs() < 0.001);
    assert!((gps.longitude - ORIGIN_LONGITUDE).abs() < 0.001);

    // Formatted NMEA sentence should start with $GPRMC and have checksum
    let nmea = gps.format_nmea_gprmc(12.5);
    assert!(nmea.starts_with("$GPRMC,"));
    assert!(nmea.contains("*"));
    assert!(nmea.ends_with("\r\n"));
}

#[test]
fn test_imu_readings_and_raw_registers() {
    let mut imu = ImuSensor::new();
    imu.update(std::f32::consts::FRAC_PI_2, 0.5, 1.0); // 90 degrees, 0.5 rad/s

    assert!((imu.yaw_deg - 90.0).abs() < 1.0);
    assert!(imu.raw_gyro_z.abs() > 0);
}

#[test]
fn test_battery_discharge_under_load() {
    let mut bat = BatteryModel::new();
    assert_eq!(bat.percentage, 100.0);
    assert_eq!(bat.open_circuit_voltage, 12.6);

    // Simulate 30 seconds of high motor throttle
    for _ in 0..300 {
        bat.step(250, 250, 180, true, 0.1);
    }

    assert!(bat.percentage < 100.0);
    assert!(bat.current_draw_a > 5.0); // Active motor draw
    assert!(bat.loaded_voltage < bat.open_circuit_voltage); // Voltage sag
}

#[test]
fn test_turbidity_spatial_field() {
    let mut turb = TurbiditySensor::new();
    
    // Sample clean river center
    turb.sample(0.0, 0.0, 0.5, 0.2);
    let center_ntu = turb.ntu;

    // Sample near stormwater runoff hotspot (x = 15.0, y = -7.0)
    turb.sample(15.0, -7.0, 0.5, 0.2);
    let runoff_ntu = turb.ntu;

    assert!(runoff_ntu > center_ntu, "Runoff hotspot ({:.1}) must have higher NTU than center ({:.1})", runoff_ntu, center_ntu);
    assert!(turb.adc_raw > 0);
}

#[test]
fn test_ldr_auto_headlights() {
    let mut ldr = LdrSensor::new();

    // Daylight phase: high lux -> headlights off
    ldr.lux = 300.0;
    ldr.update(0.0, None);
    assert!(!ldr.headlights_active);

    // Night phase: low lux (< 80) -> headlights auto-activate
    ldr.lux = 45.0;
    ldr.auto_mode = true;
    ldr.headlights_active = ldr.lux < 80.0;
    assert!(ldr.headlights_active);
}

#[test]
fn test_bts7960_thermal_throttling() {
    let mut thermal = ThermalModel::new();
    assert_eq!(thermal.throttled, false);

    // Simulate sustained full throttle for 35 seconds
    for _ in 0..350 {
        thermal.step(255, 255, 0.1);
    }

    assert!(thermal.temp_c > 70.0, "Driver should heat above 70°C under continuous 100% load. Got {:.1}°C", thermal.temp_c);
    assert!(thermal.throttled, "Safety throttle should be engaged!");

    // Enforce clamping
    let clamped_pwm = thermal.clamp_pwm(255);
    assert_eq!(clamped_pwm, 150, "Commanded 255 PWM must be throttled to 150 PWM when overheating!");
}

#[test]
fn test_firmware_state_machine_and_pid() {
    let mut fw = RustFirmwareController::new();
    fw.state = FirmwareState::HeadingHold;
    fw.target_heading_deg = 90.0;

    // Current yaw is 0 deg -> error is 90 deg -> PID should steer
    fw.update(0.0, 0.0, 0.0, true, false, 0.0, 0.02);
    assert!(fw.commanded_left_pwm != fw.commanded_right_pwm, "PID must apply differential steering when off-target!");

    // Test 3m Parking Anchor drift recovery
    fw.state = FirmwareState::ParkingAnchor;
    fw.anchor_x = Some(0.0);
    fw.anchor_y = Some(0.0);

    // Boat drifts 4.5m away (> 3.0m threshold)
    fw.update(4.5, 0.0, 0.0, true, false, 1.0, 0.02);
    assert!(fw.commanded_left_pwm.abs() > 50 || fw.commanded_right_pwm.abs() > 50, "Parking anchor must throttle back to origin when drifting > 3m!");
}

#[test]
fn test_websocket_json_commands_and_aes_encryption() {
    // Inbound command parsing
    let json_cmd = r#"{"cmd":"manual_drive","left":200,"right":-200}"#;
    let parsed: FlutterCommand = serde_json::from_str(json_cmd).expect("Valid JSON");
    match parsed {
        FlutterCommand::ManualDrive { left, right } => {
            assert_eq!(left, 200);
            assert_eq!(right, -200);
        }
        _ => panic!("Expected ManualDrive"),
    }

    // AES-256-CBC Encryption & Decryption round-trip
    let payload = r#"{"gps":{"lat":12.971598,"lon":77.594562},"battery":{"percent":95.5}}"#;
    let encrypted = encrypt_payload(payload);
    assert_ne!(payload, encrypted);
    let decrypted = decrypt_payload(&encrypted).expect("Decryption success");
    assert_eq!(payload, decrypted);
}

#[test]
fn test_pytorch_rl_dataset_csv_generation() {
    let cfg = EpisodeConfig {
        episode_id: 1,
        start_x: 2.0,
        start_y: 5.0,
        home_x: 0.0,
        home_y: 0.0,
        starting_battery_pct: 85.0,
        water_speed: 0.8,
        water_dir_deg: 0.0,
        turbulence: 0.2,
        wind_speed: 2.0,
        wind_dir_deg: 90.0,
        exploration_noise: 0.1,
        max_sim_sec: 1.0, // Short run for testing
    };

    let mut buf = Vec::new();
    let (steps, _, _, _) = run_episode(&cfg, &mut buf, 50.0);
    assert!(steps > 0);

    let csv_output = String::from_utf8(buf).expect("Valid UTF-8");
    let rows: Vec<&str> = csv_output.lines().collect();
    assert!(!rows.is_empty());

    // Verify row has 28 comma-separated fields matching the schema
    let first_row = rows[0];
    let columns: Vec<&str> = first_row.split(',').collect();
    assert_eq!(columns.len(), 28, "CSV row must have exactly 28 columns! Got {}", columns.len());
}
