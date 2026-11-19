//! Top-Down Dark-Space Environmental Renderer
//! Draws river banks, animated water vector field, foam particles, wind streams, and radio geofence rings.

use macroquad::prelude::*;
use crate::config::*;
use crate::physics::environment::EnvironmentEngine;
use crate::render::camera::SimulationCamera;

pub struct FoamParticle {
    pub x: f32,
    pub y: f32,
    pub life: f32,
}

pub struct EnvironmentalRenderer {
    pub foam_particles: Vec<FoamParticle>,
    pub wind_particles: Vec<FoamParticle>,
}

impl EnvironmentalRenderer {
    pub fn new() -> Self {
        let mut foam = Vec::new();
        let mut wind = Vec::new();

        for _ in 0..120 {
            foam.push(FoamParticle {
                x: rand::gen_range(-11.0, 11.0),
                y: rand::gen_range(-50.0, 50.0),
                life: rand::gen_range(0.0, 1.0),
            });
        }

        for _ in 0..60 {
            wind.push(FoamParticle {
                x: rand::gen_range(-30.0, 30.0),
                y: rand::gen_range(-50.0, 50.0),
                life: rand::gen_range(0.0, 1.0),
            });
        }

        Self {
            foam_particles: foam,
            wind_particles: wind,
        }
    }

    /// Step and update particle trails
    pub fn step(&mut self, env: &EnvironmentEngine, dt: f32) {
        // 1. Water foam particles
        for p in &mut self.foam_particles {
            let (vx, vy) = env.get_water_flow_at(p.x, p.y);
            p.x += vx * dt;
            p.y += vy * dt;
            p.life += dt * 0.25;

            if p.life > 1.0 || p.y > 60.0 || p.y < -60.0 || p.x.abs() > 11.5 {
                p.x = rand::gen_range(-11.0, 11.0);
                p.y = rand::gen_range(-50.0, -40.0);
                p.life = 0.0;
            }
        }

        // 2. Wind particles
        for wp in &mut self.wind_particles {
            let (w_vx, w_vy) = env.get_wind_at(wp.x, wp.y);
            wp.x += w_vx * dt;
            wp.y += w_vy * dt;
            wp.life += dt * 0.3;

            if wp.life > 1.0 || wp.x.abs() > 35.0 || wp.y.abs() > 60.0 {
                wp.x = rand::gen_range(-30.0, 30.0);
                wp.y = rand::gen_range(-50.0, 50.0);
                wp.life = 0.0;
            }
        }
    }

    /// Draw river banks, flow field, radio rings, and particles
    pub fn draw(&self, env: &EnvironmentEngine, camera: &SimulationCamera, home_x: f32, home_y: f32) {
        // Deep navy/charcoal background
        clear_background(Color::new(0.04, 0.06, 0.09, 1.0));

        let river_half_w = env.river_half_width;

        // 1. Draw River Banks
        let top_left = camera.world_to_screen(vec2(-river_half_w, 80.0));
        let btm_left = camera.world_to_screen(vec2(-river_half_w, -80.0));
        let top_right = camera.world_to_screen(vec2(river_half_w, 80.0));
        let btm_right = camera.world_to_screen(vec2(river_half_w, -80.0));

        // River water bed (subtle dark cyan gradient)
        draw_triangle(top_left, top_right, btm_right, Color::new(0.06, 0.12, 0.18, 0.7));
        draw_triangle(top_left, btm_right, btm_left, Color::new(0.06, 0.12, 0.18, 0.7));

        // Bank boundary borders
        draw_line(top_left.x, top_left.y, btm_left.x, btm_left.y, 4.0, Color::new(0.2, 0.35, 0.45, 0.9));
        draw_line(top_right.x, top_right.y, btm_right.x, btm_right.y, 4.0, Color::new(0.2, 0.35, 0.45, 0.9));

        // 2. Draw Vector Flow Arrows
        let grid_step = 4.0; // meters
        let min_world = camera.screen_to_world(vec2(0.0, screen_height()));
        let max_world = camera.screen_to_world(vec2(screen_width(), 0.0));

        let start_x = ((min_world.x / grid_step).floor() * grid_step).max(-river_half_w + 1.0);
        let end_x = ((max_world.x / grid_step).ceil() * grid_step).min(river_half_w - 1.0);
        let start_y = (min_world.y / grid_step).floor() * grid_step;
        let end_y = (max_world.y / grid_step).ceil() * grid_step;

        let mut gx = start_x;
        while gx <= end_x {
            let mut gy = start_y;
            while gy <= end_y {
                let (vx, vy) = env.get_water_flow_at(gx, gy);
                let speed = (vx * vx + vy * vy).sqrt();
                if speed > 0.05 {
                    let p_start = camera.world_to_screen(vec2(gx, gy));
                    let arrow_len = (speed * 0.75 * camera.zoom).clamp(4.0, 28.0);
                    let angle = vy.atan2(vx);
                    let p_end = vec2(p_start.x + angle.cos() * arrow_len, p_start.y - angle.sin() * arrow_len);

                    let alpha = (speed / 2.0).clamp(0.15, 0.55);
                    draw_line(p_start.x, p_start.y, p_end.x, p_end.y, 1.5, Color::new(0.15, 0.65, 0.85, alpha));
                }
                gy += grid_step;
            }
            gx += grid_step;
        }

        // 3. Draw Water Foam Particles
        for p in &self.foam_particles {
            let sp = camera.world_to_screen(vec2(p.x, p.y));
            let alpha = ((1.0 - (p.life - 0.5).abs() * 2.0).clamp(0.0, 1.0)) * 0.6;
            draw_circle(sp.x, sp.y, 2.0, Color::new(0.4, 0.85, 0.95, alpha));
        }

        // 4. Draw Wind Streaks
        for wp in &self.wind_particles {
            let sp = camera.world_to_screen(vec2(wp.x, wp.y));
            let alpha = ((1.0 - (wp.life - 0.5).abs() * 2.0).clamp(0.0, 1.0)) * 0.35;
            let (w_vx, w_vy) = env.get_wind_at(wp.x, wp.y);
            let w_ang = w_vy.atan2(w_vx);
            let streak_len = 12.0;
            let sp_end = vec2(sp.x + w_ang.cos() * streak_len, sp.y - w_ang.sin() * streak_len);
            draw_line(sp.x, sp.y, sp_end.x, sp_end.y, 1.0, Color::new(0.9, 0.85, 0.4, alpha));
        }

        // 5. Base Station & Radio Signal Coverage Rings
        let base_screen = camera.world_to_screen(vec2(BASE_STATION_X, BASE_STATION_Y));
        let r35_screen = WIFI_DROP_RADIUS_M * camera.zoom;
        let r50_screen = GEOFENCE_RADIUS_M * camera.zoom;

        // Base Station Icon
        draw_circle(base_screen.x, base_screen.y, 6.0, Color::new(0.3, 0.8, 1.0, 0.9));
        draw_circle_lines(base_screen.x, base_screen.y, 10.0, 2.0, Color::new(0.3, 0.8, 1.0, 0.5));
        draw_text("BASE STATION (0,0)", base_screen.x + 12.0, base_screen.y - 4.0, 14.0, Color::new(0.3, 0.8, 1.0, 0.8));

        // 35m Nominal Wi-Fi Limit Ring (Yellow Warning)
        draw_circle_lines(base_screen.x, base_screen.y, r35_screen, 1.5, Color::new(0.9, 0.75, 0.2, 0.35));

        // 50m Geofence Disconnect Ring (Red Severed)
        draw_circle_lines(base_screen.x, base_screen.y, r50_screen, 2.0, Color::new(1.0, 0.25, 0.25, 0.5));

        // 6. Home Waypoint Marker
        let home_screen = camera.world_to_screen(vec2(home_x, home_y));
        draw_circle(home_screen.x, home_screen.y, 5.0, Color::new(0.2, 1.0, 0.5, 0.9));
        draw_circle_lines(home_screen.x, home_screen.y, 1.5 * camera.zoom, 1.5, Color::new(0.2, 1.0, 0.5, 0.4));
        draw_text("HOME WAYPOINT", home_screen.x + 8.0, home_screen.y + 4.0, 13.0, Color::new(0.2, 1.0, 0.5, 0.8));
    }
}

