//! Dark-Space HUD Telemetry Dashboard & Diagnostics Banner

use macroquad::prelude::*;
use crate::sensors::hal::Esp32Telemetry;

pub struct HudOverlay;

impl HudOverlay {
    pub fn draw(
        telemetry: Option<&Esp32Telemetry>,
        fps: i32,
        is_cpp_loaded: bool,
        loaded_sketch_name: Option<&str>,
        compiler_msg: &str,
    ) {
        // Semi-transparent HUD background panel
        let panel_w = 340.0;
        let panel_h = 440.0;
        draw_rectangle(12.0, 12.0, panel_w, panel_h, Color::new(0.05, 0.08, 0.12, 0.88));
        draw_rectangle_lines(12.0, 12.0, panel_w, panel_h, 1.5, Color::new(0.2, 0.35, 0.5, 0.7));

        let mut y = 30.0;
        let x = 24.0;
        let line_h = 17.0;

        // Title & Header
        draw_text("RIVSIM ESP32 DIGITAL TWIN", x, y, 17.0, Color::new(0.2, 0.85, 1.0, 1.0));
        y += 18.0;
        draw_text(&format!("FPS: {} | SITL: 127.0.0.1:81 | MCP: :8000", fps), x, y, 12.0, Color::new(0.6, 0.7, 0.8, 0.9));
        y += 16.0;
        draw_line(x, y, x + panel_w - 24.0, y, 1.0, Color::new(0.2, 0.35, 0.5, 0.5));
        y += 14.0;

        if let Some(t) = telemetry {
            // Firmware State & Source
            let fw_src = if is_cpp_loaded {
                format!("C++ Host ({})", loaded_sketch_name.unwrap_or("active"))
            } else {
                "Native Rust Fallback".to_string()
            };
            draw_text(&format!("Controller: {}", fw_src), x, y, 13.0, Color::new(0.9, 0.8, 0.2, 1.0));
            y += line_h;
            draw_text(&format!("State: {} (Tgt Hdg: {:.1}°)", t.system.firmware_state, t.system.target_heading_deg), x, y, 13.0, Color::new(0.2, 1.0, 0.5, 1.0));
            y += line_h;

            // GPS Telemetry
            draw_text(&format!("GPS: {:.6}, {:.6}", t.gps.lat, t.gps.lon), x, y, 13.0, WHITE);
            y += line_h;
            draw_text(&format!("Speed: {:.1} kts ({:.2} m/s) | Sats: {}", t.gps.speed_knots, t.gps.speed_mps, t.gps.sats), x, y, 13.0, Color::new(0.7, 0.8, 0.9, 1.0));
            y += line_h;

            // IMU Telemetry
            draw_text(&format!("Yaw: {:.1}° | Gyro Z: {:.2}°/s", t.imu.yaw, t.imu.gyro_z), x, y, 13.0, WHITE);
            y += line_h;
            draw_text(&format!("Roll: {:.1}° | Pitch: {:.1}°", t.imu.roll, t.imu.pitch), x, y, 13.0, Color::new(0.7, 0.8, 0.9, 1.0));
            y += line_h;

            // Battery Bar
            draw_text(&format!("Battery: {:.1}% ({:.2}V, {:.2}A)", t.battery.percent, t.battery.voltage, t.battery.current_draw_a), x, y, 13.0, WHITE);
            y += 14.0;
            // Draw battery gauge bar
            let bar_w = 200.0;
            let bar_h = 8.0;
            draw_rectangle(x, y, bar_w, bar_h, Color::new(0.2, 0.25, 0.3, 0.8));
            let fill_w = (t.battery.percent / 100.0).clamp(0.0, 1.0) * bar_w;
            let bat_color = if t.battery.percent > 40.0 {
                Color::new(0.2, 0.9, 0.4, 0.9)
            } else if t.battery.percent > 18.0 {
                Color::new(0.9, 0.8, 0.2, 0.9)
            } else {
                Color::new(1.0, 0.2, 0.2, 0.9)
            };
            draw_rectangle(x, y, fill_w, bar_h, bat_color);
            y += 14.0;

            // Actuators & Thermal Throttling
            draw_text(&format!("Motors: L: {} | R: {} | Conv: {}", t.actuators.left_pwm, t.actuators.right_pwm, t.actuators.conveyor_pwm), x, y, 13.0, WHITE);
            y += line_h;

            let temp_color = if t.actuators.driver_throttled {
                Color::new(1.0, 0.25, 0.25, 1.0)
            } else {
                Color::new(0.8, 0.85, 0.9, 1.0)
            };
            let throttle_tag = if t.actuators.driver_throttled { " [OVERHEAT CLAMP]" } else { "" };
            draw_text(&format!("Driver Temp: {:.1}°C{}", t.actuators.driver_temp_c, throttle_tag), x, y, 13.0, temp_color);
            y += line_h;

            // Water Turbidity & LDR Headlights
            draw_text(&format!("Turbidity: {:.1} NTU | Light: {}", t.water_quality.turbidity_ntu, if t.actuators.headlights { "LIGHTS ON" } else { "LIGHTS OFF" }), x, y, 13.0, Color::new(0.4, 0.9, 0.8, 1.0));
            y += line_h;

            // Radio & Failsafe
            let radio_color = if t.system.link_connected {
                Color::new(0.2, 0.9, 0.4, 1.0)
            } else {
                Color::new(1.0, 0.3, 0.3, 1.0)
            };
            let radio_status = if t.system.link_connected {
                format!("Wi-Fi: Connected (RSSI: {} dBm)", t.system.rssi)
            } else {
                "Wi-Fi: SEVERED [FAILSAFE ENGAGED]".to_string()
            };
            draw_text(&radio_status, x, y, 13.0, radio_color);
            y += line_h;
        }

        // Compiler / Message Banner
        if !compiler_msg.is_empty() {
            y += 4.0;
            draw_text(&format!("Note: {}", compiler_msg), x, y, 11.0, Color::new(0.9, 0.7, 0.2, 0.9));
        }

        // Bottom Screen Control Instructions Bar
        let bar_y = screen_height() - 28.0;
        draw_rectangle(0.0, bar_y - 4.0, screen_width(), 32.0, Color::new(0.04, 0.07, 0.1, 0.92));
        draw_line(0.0, bar_y - 4.0, screen_width(), bar_y - 4.0, 1.0, Color::new(0.2, 0.3, 0.4, 0.6));
        draw_text(
            "[WASD/Arrows]: Drive | [Space]: Stop | [C]: Conveyor | [H]: Headlights | [F]: Follow Cam | [L]: Load C++ Sketch | [1-4]: Modes",
            16.0,
            bar_y + 16.0,
            13.0,
            Color::new(0.7, 0.8, 0.9, 0.9),
        );
    }
}

