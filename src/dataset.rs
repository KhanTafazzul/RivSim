//! PyTorch Reinforcement Learning Return-to-Home CSV Dataset Generator
//! Logs 28-column episode trajectories for training energy-optimized navigation policies.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use rand::Rng;

use crate::physics::engine::PhysicsEngine;
use crate::sensors::hal::HardwareAbstractionLayer;

pub const CSV_HEADER: &str = "episode_id,step,sim_time_s,boat_x,boat_y,home_x,home_y,dist_to_home_m,bearing_to_home_deg,yaw_deg,vx,vy,angular_vel,water_vx,water_vy,wind_vx,wind_vy,battery_pct,battery_v,driver_temp_c,left_pwm,right_pwm,delta_dist_m,battery_drop_step,step_reward,reached_home,battery_dead,geofence_breach\n";

#[derive(Debug, Clone)]
pub struct EpisodeConfig {
    pub episode_id: u32,
    pub start_x: f32,
    pub start_y: f32,
    pub home_x: f32,
    pub home_y: f32,
    pub starting_battery_pct: f32,
    pub water_speed: f32,
    pub water_dir_deg: f32,
    pub turbulence: f32,
    pub wind_speed: f32,
    pub wind_dir_deg: f32,
    pub exploration_noise: f32,
    pub max_sim_sec: f32,
}

impl EpisodeConfig {
    pub fn random(episode_id: u32, enable_exploration: bool) -> Self {
        let mut rng = rand::thread_rng();
        Self {
            episode_id,
            start_x: rng.gen_range(-9.0..9.0),
            start_y: rng.gen_range(-40.0..40.0),
            home_x: rng.gen_range(-3.0..3.0),
            home_y: rng.gen_range(-5.0..5.0),
            starting_battery_pct: rng.gen_range(15.0..100.0),
            water_speed: rng.gen_range(0.2..2.2),
            water_dir_deg: rng.gen_range(0.0..360.0),
            turbulence: rng.gen_range(0.1..0.8),
            wind_speed: rng.gen_range(0.0..12.0),
            wind_dir_deg: rng.gen_range(0.0..360.0),
            exploration_noise: if enable_exploration { rng.gen_range(0.05..0.35) } else { 0.0 },
            max_sim_sec: 120.0,
        }
    }
}

/// Run a single return-to-home simulation episode and append rows to a CSV writer
pub fn run_episode<W: Write>(
    cfg: &EpisodeConfig,
    writer: &mut W,
    step_hz: f32,
) -> (u32, bool, bool, bool) {
    let dt = 1.0 / step_hz;
    let mut physics = PhysicsEngine::new(cfg.start_x, cfg.start_y, 0.0);
    let mut hal = HardwareAbstractionLayer::new();

    physics.env.set_water_current(cfg.water_speed, cfg.water_dir_deg, cfg.turbulence);
    physics.env.set_weather(cfg.wind_speed, cfg.wind_dir_deg, 0.2);
    hal.battery.set_percentage(cfg.starting_battery_pct);

    hal.firmware.home_x = cfg.home_x;
    hal.firmware.home_y = cfg.home_y;
    hal.firmware.state = crate::sensors::firmware::FirmwareState::ReturnToHome;

    let mut step = 0u32;
    let mut sim_time_s = 0.0f32;
    let mut prev_dist = ((cfg.start_x - cfg.home_x).powi(2) + (cfg.start_y - cfg.home_y).powi(2)).sqrt();
    let mut prev_battery_pct = cfg.starting_battery_pct;

    let mut reached_home = false;
    let mut battery_dead = false;
    let mut geofence_breach = false;

    let mut rng = rand::thread_rng();

    while sim_time_s < cfg.max_sim_sec {
        let (bx, by, yaw_rad) = physics.get_boat_pose();
        let (vx, vy, omega_z) = physics.get_boat_velocities();
        let yaw_deg = (yaw_rad.to_degrees() % 360.0 + 360.0) % 360.0;

        let dx = cfg.home_x - bx;
        let dy = cfg.home_y - by;
        let dist_to_home = (dx * dx + dy * dy).sqrt();
        let bearing_to_home = (dy.atan2(dx).to_degrees() % 360.0 + 360.0) % 360.0;

        // Update virtual sensors
        hal.gps.update(bx, by, vx, vy, yaw_deg, dt);
        hal.imu.update(yaw_rad, omega_z, sim_time_s);
        hal.battery.step(
            physics.boat.left_paddle_pwm,
            physics.boat.right_paddle_pwm,
            physics.boat.conveyor_pwm,
            hal.ldr.headlights_active,
            dt,
        );
        hal.thermal.step(physics.boat.left_paddle_pwm, physics.boat.right_paddle_pwm, dt);

        // Run firmware control loop
        hal.firmware.update(
            bx,
            by,
            yaw_deg,
            !physics.radio.connection_severed,
            hal.thermal.throttled,
            sim_time_s,
            dt,
        );

        // Optional exploration noise for RL domain randomization
        let mut left_pwm = hal.firmware.commanded_left_pwm;
        let mut right_pwm = hal.firmware.commanded_right_pwm;
        if cfg.exploration_noise > 0.0 {
            let noise_l = (rng.gen_range(-1.0..1.0) * cfg.exploration_noise * 100.0) as i16;
            let noise_r = (rng.gen_range(-1.0..1.0) * cfg.exploration_noise * 100.0) as i16;
            left_pwm = (left_pwm + noise_l).clamp(-255, 255);
            right_pwm = (right_pwm + noise_r).clamp(-255, 255);
        }

        physics.boat.left_paddle_pwm = left_pwm;
        physics.boat.right_paddle_pwm = right_pwm;

        // Step physics world
        physics.step(dt);

        let (water_vx, water_vy) = physics.env.get_water_flow_at(bx, by);
        let (wind_vx, wind_vy) = physics.env.get_wind_at(bx, by);

        // Metrics & Reward Calculation
        let delta_dist = prev_dist - dist_to_home; // Positive if getting closer
        let battery_drop_step = (prev_battery_pct - hal.battery.percentage).max(0.0);
        prev_dist = dist_to_home;
        prev_battery_pct = hal.battery.percentage;

        // Termination checks
        reached_home = dist_to_home <= 1.5;
        battery_dead = hal.battery.is_depleted();
        geofence_breach = physics.radio.geofence_breached;

        let mut step_reward = (delta_dist * 12.0) - (battery_drop_step * 2.5);
        if reached_home {
            step_reward += 100.0;
        } else if battery_dead {
            step_reward -= 100.0;
        } else if geofence_breach {
            step_reward -= 100.0;
        }

        // Write row
        let _ = writeln!(
            writer,
            "{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{:.2},{:.1},{},{},{:.4},{:.4},{:.3},{},{},{}",
            cfg.episode_id,
            step,
            sim_time_s,
            bx,
            by,
            cfg.home_x,
            cfg.home_y,
            dist_to_home,
            bearing_to_home,
            yaw_deg,
            vx,
            vy,
            omega_z,
            water_vx,
            water_vy,
            wind_vx,
            wind_vy,
            hal.battery.percentage,
            hal.battery.loaded_voltage,
            hal.thermal.temp_c,
            left_pwm,
            right_pwm,
            delta_dist,
            battery_drop_step,
            step_reward,
            if reached_home { 1 } else { 0 },
            if battery_dead { 1 } else { 0 },
            if geofence_breach { 1 } else { 0 },
        );

        step += 1;
        sim_time_s += dt;

        if reached_home || battery_dead || geofence_breach {
            break;
        }
    }

    (step, reached_home, battery_dead, geofence_breach)
}

/// Create or open a CSV file and write header
pub fn create_csv_dataset<P: AsRef<Path>>(path: P) -> std::io::Result<BufWriter<File>> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writer.write_all(CSV_HEADER.as_bytes())?;
    Ok(writer)
}

