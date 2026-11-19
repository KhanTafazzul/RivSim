//! RivSim: 2D Digital Twin Physics Simulator for Autonomous River-Cleaning Catamaran
//! Pure Rust implementation featuring Rapier2D physics, Tokio async WebSocket & MCP,
//! ESP32 C++ firmware execution host, and PyTorch RL dataset generator.

use std::sync::Arc;
use std::thread;
use clap::Parser;
use crossbeam_channel::unbounded;
use macroquad::prelude::*;
use tokio::sync::RwLock;

mod config;
mod dataset;
mod firmware_host;
mod headless;
mod network;
mod physics;
mod sensors;
mod render;
use crate::firmware_host::loader::DynamicFirmwareHost;
use crate::headless::HeadlessBatchRunner;
use crate::network::mcp_server::{start_mcp_server, McpServerState};
use crate::network::ws_server::{start_websocket_server, SimulationCommand, WsServerState};
use crate::physics::engine::PhysicsEngine;
use crate::render::boat_view::BoatRenderer;
use crate::render::camera::SimulationCamera;
use crate::render::hud::HudOverlay;
use crate::render::renderer::EnvironmentalRenderer;
use crate::sensors::firmware::FirmwareState;
use crate::sensors::hal::HardwareAbstractionLayer;

#[derive(Parser, Debug)]
#[command(author, version, about = "RivSim: Autonomous Catamaran Digital Twin Physics Engine")]
struct CliArgs {
    /// Simulation mode: 'visual' (GUI + SITL WebSocket + MCP) or 'headless' (High-throughput Rayon RL dataset)
    #[arg(short, long, default_value = "visual")]
    mode: String,

    /// Number of episodes to simulate in headless mode
    #[arg(short, long, default_value_t = 100)]
    episodes: u32,

    /// Target CSV rows in headless mode
    #[arg(short, long, default_value_t = 50000)]
    rows: u32,

    /// Output CSV file path
    #[arg(short, long, default_value = "return_dataset.csv")]
    out: String,

    /// Enable exploration noise during headless episodes
    #[arg(long, default_value_t = true)]
    explore: bool,

    /// Optional path to an ESP32 C++ firmware sketch (.cpp / .ino) to compile and hot-load
    #[arg(short, long)]
    firmware: Option<String>,
}

fn window_conf() -> Conf {
    Conf {
        window_title: "RivSim - Autonomous Catamaran Digital Twin (ESP32 SITL & MCP)".to_string(),
        window_width: 1280,
        window_height: 720,
        high_dpi: true,
        sample_count: 4,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let args = CliArgs::parse();

    // Mode 1: Parallel High-Speed Headless RL Dataset Generator
    if args.mode == "headless" {
        HeadlessBatchRunner::run_batch(args.episodes, args.rows, &args.out, args.explore);
        return;
    }

    // Mode 2: Interactive Visualizer + Tokio SITL WebSocket + Native MCP Server
    println!("[RivSim] Starting Visual SITL mode...");

    let (cmd_tx, cmd_rx) = unbounded::<SimulationCommand>();
    let latest_telemetry = Arc::new(RwLock::new(None));
    let drop_packets = Arc::new(RwLock::new(false));
    let connection_severed = Arc::new(RwLock::new(false));

    let (shutdown_tx, _) = tokio::sync::broadcast::channel::<()>(1);

    // Spawn Background Tokio Runtime for WebSocket (127.0.0.1:81) and MCP (127.0.0.1:8000/mcp)
    let ws_state = Arc::new(WsServerState {
        latest_telemetry: latest_telemetry.clone(),
        cmd_sender: cmd_tx.clone(),
        drop_packets: drop_packets.clone(),
        connection_severed: connection_severed.clone(),
    });

    let mcp_state = McpServerState {
        cmd_sender: cmd_tx.clone(),
        latest_telemetry: latest_telemetry.clone(),
    };

    let shutdown_rx_ws = shutdown_tx.subscribe();
    let shutdown_rx_mcp = shutdown_tx.subscribe();

    thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .expect("Failed to build background Tokio runtime");

        rt.block_on(async {
            tokio::spawn(start_websocket_server(ws_state, shutdown_rx_ws));
            tokio::spawn(start_mcp_server(mcp_state, shutdown_rx_mcp));
            futures_util::future::pending::<()>().await;
        });
    });

    // Initialize Simulation Core
    let mut physics = PhysicsEngine::new(0.0, -10.0, 0.0);
    let mut hal = HardwareAbstractionLayer::new();
    let mut camera = SimulationCamera::new();
    let mut env_renderer = EnvironmentalRenderer::new();
    let mut firmware_host = DynamicFirmwareHost::new();

    let mut compiler_msg = String::new();

    // Attempt to load firmware sketch if specified via CLI
    if let Some(ref fw_path) = args.firmware {
        match firmware_host.compile_and_load(fw_path) {
            Ok(_) => compiler_msg = format!("Loaded C++ sketch: {}", fw_path),
            Err(e) => compiler_msg = format!("C++ compile note: {}", e),
        }
    }

    // Default home coordinates
    let mut home_x = 0.0f32;
    let mut home_y = 0.0f32;

    let mut manual_override = false;
    let mut conveyor_toggle = false;
    let mut headlights_manual: Option<bool> = None;

    // Main Interactive Simulation Loop
    loop {
        let dt = get_frame_time().clamp(0.001, 0.05);

        // 1. Process Inbound Commands from WebSocket (Flutter) and MCP
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                SimulationCommand::ManualDrive { left, right } => {
                    manual_override = true;
                    hal.firmware.state = FirmwareState::Manual;
                    hal.firmware.commanded_left_pwm = left;
                    hal.firmware.commanded_right_pwm = right;
                }
                SimulationCommand::Conveyor { speed } => {
                    hal.firmware.commanded_conveyor_pwm = speed;
                    physics.boat.conveyor_pwm = speed;
                }
                SimulationCommand::SetAnchor { active } => {
                    if active {
                        hal.firmware.state = FirmwareState::ParkingAnchor;
                        hal.firmware.anchor_x = None;
                        hal.firmware.anchor_y = None;
                    } else {
                        hal.firmware.state = FirmwareState::HeadingHold;
                    }
                }
                SimulationCommand::SetHeading { deg } => {
                    hal.firmware.state = FirmwareState::HeadingHold;
                    hal.firmware.target_heading_deg = deg;
                }
                SimulationCommand::AutoReturn => {
                    hal.firmware.state = FirmwareState::ReturnToHome;
                }
                SimulationCommand::Headlights { active } => {
                    headlights_manual = Some(active);
                }
                SimulationCommand::SetWaterCurrent { speed_mps, dir_deg, turbulence } => {
                    physics.env.set_water_current(speed_mps, dir_deg, turbulence);
                }
                SimulationCommand::SetWeather { wind_speed_mps, wind_dir_deg, gust_prob } => {
                    physics.env.set_weather(wind_speed_mps, wind_dir_deg, gust_prob);
                }
                SimulationCommand::InjectFault { fault_type, _duration_sec: _ } => {
                    match fault_type.as_str() {
                        "motor_jam_left" => physics.boat.fault_motor_jam_left = true,
                        "motor_jam_right" => physics.boat.fault_motor_jam_right = true,
                        "gps_spike" => hal.gps.fault_spike = true,
                        "wifi_drop" => physics.radio.fault_wifi_drop = true,
                        "clear" => {
                            physics.boat.fault_motor_jam_left = false;
                            physics.boat.fault_motor_jam_right = false;
                            hal.gps.fault_spike = false;
                            physics.radio.fault_wifi_drop = false;
                        }
                        _ => {}
                    }
                }
                SimulationCommand::TeleportBoat { x, y, yaw_deg, battery_pct } => {
                    physics.teleport_boat(x, y, yaw_deg);
                    hal.battery.set_percentage(battery_pct);
                }
                SimulationCommand::SetHomeCoordinates { x, y } => {
                    home_x = x;
                    home_y = y;
                    hal.firmware.home_x = x;
                    hal.firmware.home_y = y;
                }
                SimulationCommand::UploadFirmware { source_path } => {
                    match firmware_host.compile_and_load(&source_path) {
                        Ok(_) => compiler_msg = format!("Loaded C++ sketch: {}", source_path),
                        Err(e) => compiler_msg = format!("C++ compile note: {}", e),
                    }
                }
            }
        }

        // 2. Keyboard Direct Drive & Control Hotkeys
        let mut key_throttle: i16 = 0;
        let mut key_steer: i16 = 0;

        if is_key_down(KeyCode::W) || is_key_down(KeyCode::Up) {
            key_throttle += 200;
        }
        if is_key_down(KeyCode::S) || is_key_down(KeyCode::Down) {
            key_throttle -= 180;
        }
        if is_key_down(KeyCode::A) || is_key_down(KeyCode::Left) {
            key_steer -= 120;
        }
        if is_key_down(KeyCode::D) || is_key_down(KeyCode::Right) {
            key_steer += 120;
        }
        if is_key_pressed(KeyCode::Space) {
            manual_override = true;
            hal.firmware.state = FirmwareState::Manual;
            hal.firmware.commanded_left_pwm = 0;
            hal.firmware.commanded_right_pwm = 0;
        }

        if key_throttle != 0 || key_steer != 0 {
            manual_override = true;
            hal.firmware.state = FirmwareState::Manual;
            hal.firmware.commanded_left_pwm = (key_throttle - key_steer).clamp(-255, 255);
            hal.firmware.commanded_right_pwm = (key_throttle + key_steer).clamp(-255, 255);
        }

        // Toggle Conveyor ('C')
        if is_key_pressed(KeyCode::C) {
            conveyor_toggle = !conveyor_toggle;
            let spd = if conveyor_toggle { 180 } else { 0 };
            hal.firmware.commanded_conveyor_pwm = spd;
            physics.boat.conveyor_pwm = spd;
        }

        // Toggle Headlights ('H')
        if is_key_pressed(KeyCode::H) {
            let cur = headlights_manual.unwrap_or(hal.ldr.headlights_active);
            headlights_manual = Some(!cur);
        }

        // Mode Switching Hotkeys (1-4)
        if is_key_pressed(KeyCode::Key1) {
            hal.firmware.state = FirmwareState::HeadingHold;
            manual_override = false;
        } else if is_key_pressed(KeyCode::Key2) {
            hal.firmware.state = FirmwareState::ParkingAnchor;
            hal.firmware.anchor_x = None;
            hal.firmware.anchor_y = None;
            manual_override = false;
        } else if is_key_pressed(KeyCode::Key3) {
            hal.firmware.state = FirmwareState::Loiter;
            manual_override = false;
        } else if is_key_pressed(KeyCode::Key4) {
            hal.firmware.state = FirmwareState::ReturnToHome;
            manual_override = false;
        }

        // Hot-load sample C++ sketch on 'L'
        if is_key_pressed(KeyCode::L) {
            let sketch_path = "firmware/boat_firmware.cpp";
            match firmware_host.compile_and_load(sketch_path) {
                Ok(_) => compiler_msg = format!("Loaded C++ sketch: {}", sketch_path),
                Err(e) => compiler_msg = format!("C++ compile note: {}", e),
            }
        }

        // 3. Query Physics State
        let (bx, by, yaw_rad) = physics.get_boat_pose();
        let (vx, vy, omega_z) = physics.get_boat_velocities();
        let yaw_deg = (yaw_rad.to_degrees() % 360.0 + 360.0) % 360.0;

        // 4. Update Hardware Sensors
        hal.gps.update(bx, by, vx, vy, yaw_deg, dt);
        hal.imu.update(yaw_rad, omega_z, physics.sim_time_s);
        hal.turbidity.sample(bx, by, (vx * vx + vy * vy).sqrt(), physics.env.turbulence_intensity);
        hal.ldr.update(physics.sim_time_s, headlights_manual);
        hal.battery.step(
            physics.boat.left_paddle_pwm,
            physics.boat.right_paddle_pwm,
            physics.boat.conveyor_pwm,
            hal.ldr.headlights_active,
            dt,
        );
        hal.thermal.step(physics.boat.left_paddle_pwm, physics.boat.right_paddle_pwm, dt);

        // 5. Controller Execution (C++ Dynamic Host vs Native Rust Fallback)
        if firmware_host.is_loaded {
            let nmea = hal.gps.format_nmea_gprmc(physics.sim_time_s);
            firmware_host.bridge_state.populate_inputs(
                physics.sim_time_s,
                hal.turbidity.adc_raw,
                hal.battery.adc_raw,
                hal.ldr.adc_raw,
                hal.thermal.adc_raw,
                hal.imu.raw_gyro_z,
                hal.imu.raw_accel_x,
                hal.imu.raw_accel_y,
                &nmea,
                physics.radio.connection_severed,
            );

            // Execute C++ sketch loop()
            firmware_host.step();

            if !manual_override {
                let clamped_left = hal.thermal.clamp_pwm(firmware_host.bridge_state.pwm_motor_left);
                let clamped_right = hal.thermal.clamp_pwm(firmware_host.bridge_state.pwm_motor_right);
                physics.boat.left_paddle_pwm = clamped_left;
                physics.boat.right_paddle_pwm = clamped_right;
                physics.boat.conveyor_pwm = firmware_host.bridge_state.pwm_conveyor;
            }
        } else {
            // Run Native Rust Fallback Firmware
            hal.firmware.home_x = home_x;
            hal.firmware.home_y = home_y;
            if !manual_override {
                hal.firmware.update(
                    bx,
                    by,
                    yaw_deg,
                    !physics.radio.connection_severed,
                    hal.thermal.throttled,
                    physics.sim_time_s,
                    dt,
                );
                physics.boat.left_paddle_pwm = hal.firmware.commanded_left_pwm;
                physics.boat.right_paddle_pwm = hal.firmware.commanded_right_pwm;
            } else {
                physics.boat.left_paddle_pwm = hal.thermal.clamp_pwm(hal.firmware.commanded_left_pwm);
                physics.boat.right_paddle_pwm = hal.thermal.clamp_pwm(hal.firmware.commanded_right_pwm);
            }
        }

        // 6. Step Rapier2D Physics World
        physics.step(dt);

        // 7. Update Radio Status for WebSocket Thread
        let should_drop = physics.radio.should_drop_packet();
        let is_severed = physics.radio.connection_severed;
        {
            if let Ok(mut locked_drop) = drop_packets.try_write() {
                *locked_drop = should_drop;
            }
            if let Ok(mut locked_severed) = connection_severed.try_write() {
                *locked_severed = is_severed;
            }
        }

        // 8. Update Telemetry Cache for WebSocket & MCP
        let current_telemetry = hal.build_telemetry_json(
            physics.sim_time_s,
            physics.radio.rssi_dbm,
            !is_severed,
        );
        {
            if let Ok(mut locked_tel) = latest_telemetry.try_write() {
                *locked_tel = Some(current_telemetry.clone());
            }
        }

        // 9. Update Viewport Camera & Particle Simulation
        camera.update(vec2(bx, by));
        env_renderer.step(&physics.env, dt);

        // 10. Render Frame
        env_renderer.draw(&physics.env, &camera, home_x, home_y);
        BoatRenderer::draw(
            &physics.boat,
            &camera,
            bx,
            by,
            yaw_rad,
            vec2(vx, vy),
            hal.ldr.headlights_active,
        );
        HudOverlay::draw(
            Some(&current_telemetry),
            get_fps(),
            firmware_host.is_loaded,
            firmware_host.loaded_source.as_deref(),
            &compiler_msg,
        );

        next_frame().await;
    }
}
