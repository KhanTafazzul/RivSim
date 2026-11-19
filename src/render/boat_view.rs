//! Catamaran 2D Visual Rendering
//! Accurately draws PVC pontoons, wide front conveyor belt, rotating paddles, and headlights.

use macroquad::prelude::*;
use crate::config::*;
use crate::physics::boat::BoatModel;
use crate::render::camera::SimulationCamera;

pub struct BoatRenderer;

impl BoatRenderer {
    pub fn draw(
        boat: &BoatModel,
        camera: &SimulationCamera,
        world_x: f32,
        world_y: f32,
        yaw_rad: f32,
        linvel: Vec2,
        headlights_active: bool,
    ) {
        let screen_pos = camera.world_to_screen(vec2(world_x, world_y));
        let zoom = camera.zoom;

        // Heading angles in screen space (accounting for inverted Y)
        let screen_yaw = -yaw_rad;
        let cos_r = screen_yaw.cos();
        let sin_r = screen_yaw.sin();

        // Helper closure to transform local boat offsets (meters) to screen pixels
        let to_screen = |lx: f32, ly: f32| -> Vec2 {
            // Local frame: X right, Y forward
            let rx = lx * cos_r - ly * sin_r;
            let ry = lx * sin_r + ly * cos_r;
            vec2(screen_pos.x + rx * zoom, screen_pos.y - ry * zoom)
        };

        // 1. Auto-Headlights Light Cones
        if headlights_active {
            let left_headlight = to_screen(-PONTOON_SPACING_M * 0.45, PONTOON_LENGTH_M * 0.55);
            let right_headlight = to_screen(PONTOON_SPACING_M * 0.45, PONTOON_LENGTH_M * 0.55);
            let beam_dist = 6.5; // meters
            let beam_spread = 2.0;

            let beam_l1 = to_screen(-PONTOON_SPACING_M * 0.45 - beam_spread, PONTOON_LENGTH_M * 0.55 + beam_dist);
            let beam_l2 = to_screen(-PONTOON_SPACING_M * 0.45 + beam_spread, PONTOON_LENGTH_M * 0.55 + beam_dist);
            let beam_r1 = to_screen(PONTOON_SPACING_M * 0.45 - beam_spread, PONTOON_LENGTH_M * 0.55 + beam_dist);
            let beam_r2 = to_screen(PONTOON_SPACING_M * 0.45 + beam_spread, PONTOON_LENGTH_M * 0.55 + beam_dist);

            draw_triangle(left_headlight, beam_l1, beam_l2, Color::new(1.0, 0.95, 0.6, 0.12));
            draw_triangle(right_headlight, beam_r1, beam_r2, Color::new(1.0, 0.95, 0.6, 0.12));
        }

        let half_pontoon_len = PONTOON_LENGTH_M * 0.5;
        let pontoon_r = PONTOON_DIAMETER_M * 0.5;
        let half_spacing = PONTOON_SPACING_M * 0.5;

        // 2. Twin Cylindrical PVC Pontoons with End Caps
        for side in &[-1.0f32, 1.0f32] {
            let px = side * half_spacing;
            let p_front = to_screen(px, half_pontoon_len - pontoon_r);
            let p_rear = to_screen(px, -half_pontoon_len + pontoon_r);
            let tube_thickness = PONTOON_DIAMETER_M * zoom;

            // Main cylindrical PVC tube body (light pearl gray)
            draw_line(p_rear.x, p_rear.y, p_front.x, p_front.y, tube_thickness, Color::new(0.88, 0.91, 0.94, 0.95));

            // Inner pontoon outline
            draw_line(p_rear.x, p_rear.y, p_front.x, p_front.y, tube_thickness * 0.85, Color::new(0.72, 0.76, 0.82, 0.95));

            // Front rounded end cap
            let cap_front = to_screen(px, half_pontoon_len - pontoon_r);
            draw_circle(cap_front.x, cap_front.y, pontoon_r * zoom, Color::new(0.95, 0.97, 1.0, 1.0));

            // Rear rounded end cap
            let cap_rear = to_screen(px, -half_pontoon_len + pontoon_r);
            draw_circle(cap_rear.x, cap_rear.y, pontoon_r * zoom, Color::new(0.65, 0.68, 0.74, 1.0));
        }

        // Cross Struts connecting pontoons
        let strut_f_l = to_screen(-half_spacing, half_pontoon_len * 0.3);
        let strut_f_r = to_screen(half_spacing, half_pontoon_len * 0.3);
        draw_line(strut_f_l.x, strut_f_l.y, strut_f_r.x, strut_f_r.y, 4.0, Color::new(0.3, 0.35, 0.4, 1.0));

        let strut_r_l = to_screen(-half_spacing, -half_pontoon_len * 0.4);
        let strut_r_r = to_screen(half_spacing, -half_pontoon_len * 0.4);
        draw_line(strut_r_l.x, strut_r_l.y, strut_r_r.x, strut_r_r.y, 4.0, Color::new(0.3, 0.35, 0.4, 1.0));

        // 3. Front Conveyor Belt (Extends 1" wider on each side)
        let conv_w = CONVEYOR_WIDTH_M;
        let conv_half_w = conv_w * 0.5;
        let conv_front_y = half_pontoon_len + CONVEYOR_LENGTH_M * 0.5;
        let conv_rear_y = half_pontoon_len - CONVEYOR_LENGTH_M * 0.5;

        let c_fl = to_screen(-conv_half_w, conv_front_y);
        let c_fr = to_screen(conv_half_w, conv_front_y);
        let c_rr = to_screen(conv_half_w, conv_rear_y);
        let c_rl = to_screen(-conv_half_w, conv_rear_y);

        // Conveyor belt bed (emerald teal)
        draw_triangle(c_fl, c_fr, c_rr, Color::new(0.08, 0.55, 0.45, 0.9));
        draw_triangle(c_fl, c_rr, c_rl, Color::new(0.08, 0.55, 0.45, 0.9));

        // Animated Conveyor Slats
        let num_slats = 6;
        for i in 0..num_slats {
            let offset_phase = ((boat.conveyor_offset + (i as f32 / num_slats as f32)) % 1.0).fract();
            let slat_y = conv_rear_y + offset_phase * CONVEYOR_LENGTH_M;
            let s_left = to_screen(-conv_half_w + 0.02, slat_y);
            let s_right = to_screen(conv_half_w - 0.02, slat_y);
            draw_line(s_left.x, s_left.y, s_right.x, s_right.y, 2.5, Color::new(0.2, 0.8, 0.65, 0.8));
        }

        // Conveyor outer safety side rails
        draw_line(c_fl.x, c_fl.y, c_rl.x, c_rl.y, 3.0, Color::new(0.9, 0.6, 0.1, 1.0));
        draw_line(c_fr.x, c_fr.y, c_rr.x, c_rr.y, 3.0, Color::new(0.9, 0.6, 0.1, 1.0));

        // 4. Waste Box & Electronics Enclosure
        let box_l = to_screen(-half_spacing * 0.75 + COG_OFFSET_X, 0.15 + COG_OFFSET_Y);
        let box_r = to_screen(half_spacing * 0.75 + COG_OFFSET_X, 0.15 + COG_OFFSET_Y);
        let box_br = to_screen(half_spacing * 0.75 + COG_OFFSET_X, -0.35 + COG_OFFSET_Y);
        let box_bl = to_screen(-half_spacing * 0.75 + COG_OFFSET_X, -0.35 + COG_OFFSET_Y);

        draw_triangle(box_l, box_r, box_br, Color::new(0.18, 0.22, 0.28, 0.95));
        draw_triangle(box_l, box_br, box_bl, Color::new(0.18, 0.22, 0.28, 0.95));

        // Enclosure border
        draw_line(box_l.x, box_l.y, box_r.x, box_r.y, 2.0, Color::new(0.4, 0.48, 0.56, 1.0));
        draw_line(box_r.x, box_r.y, box_br.x, box_br.y, 2.0, Color::new(0.4, 0.48, 0.56, 1.0));
        draw_line(box_br.x, box_br.y, box_bl.x, box_bl.y, 2.0, Color::new(0.4, 0.48, 0.56, 1.0));
        draw_line(box_bl.x, box_bl.y, box_l.x, box_l.y, 2.0, Color::new(0.4, 0.48, 0.56, 1.0));

        // ESP32 Status LED (blinks green)
        let led_pos = to_screen(0.0 + COG_OFFSET_X, -0.1 + COG_OFFSET_Y);
        let led_color = if (get_time() * 2.0).fract() > 0.5 {
            Color::new(0.1, 1.0, 0.3, 1.0)
        } else {
            Color::new(0.05, 0.4, 0.15, 0.8)
        };
        draw_circle(led_pos.x, led_pos.y, 3.5, led_color);

        // Center of Gravity Marker
        let cog_pos = to_screen(COG_OFFSET_X, COG_OFFSET_Y);
        draw_circle(cog_pos.x, cog_pos.y, 2.5, Color::new(1.0, 0.3, 0.3, 0.9));

        // 5. Dual Paddle Wheels (Rear differential drive)
        let half_paddle_span = PADDLE_SPACING_M * 0.5;
        let paddle_y = -PONTOON_LENGTH_M * 0.35;
        let paddle_r = PADDLE_DIAMETER_M * 0.5;

        // Left Paddle
        Self::draw_paddle_wheel(
            to_screen(-half_paddle_span, paddle_y),
            paddle_r * zoom,
            boat.left_wheel_angle,
            boat.left_paddle_pwm,
            boat.fault_motor_jam_left,
        );

        // Right Paddle
        Self::draw_paddle_wheel(
            to_screen(half_paddle_span, paddle_y),
            paddle_r * zoom,
            boat.right_wheel_angle,
            boat.right_paddle_pwm,
            boat.fault_motor_jam_right,
        );

        // 6. Velocity Vector Indicator
        let v_len = linvel.length();
        if v_len > 0.1 {
            let vel_screen = vec2(linvel.x, -linvel.y) * zoom * 0.8;
            draw_line(
                screen_pos.x,
                screen_pos.y,
                screen_pos.x + vel_screen.x,
                screen_pos.y + vel_screen.y,
                2.0,
                Color::new(0.2, 0.9, 1.0, 0.7),
            );
        }
    }

    fn draw_paddle_wheel(center: Vec2, radius: f32, angle: f32, pwm: i16, is_jammed: bool) {
        let wheel_color = if is_jammed {
            Color::new(0.9, 0.2, 0.2, 0.9) // Red if jammed
        } else {
            Color::new(0.25, 0.3, 0.38, 0.9)
        };

        // Outer rim
        draw_circle_lines(center.x, center.y, radius, 2.0, wheel_color);
        draw_circle(center.x, center.y, radius * 0.35, wheel_color);

        // Rotating blades
        let num_blades = 6;
        for i in 0..num_blades {
            let blade_ang = angle + (i as f32 * std::f32::consts::TAU / num_blades as f32);
            let b_end = vec2(center.x + blade_ang.cos() * radius, center.y + blade_ang.sin() * radius);
            draw_line(center.x, center.y, b_end.x, b_end.y, 2.5, Color::new(0.85, 0.9, 0.95, 0.9));
        }

        // Throttle indicator dot
        if pwm.abs() > 10 {
            let dot_color = if pwm > 0 { Color::new(0.2, 1.0, 0.4, 0.9) } else { Color::new(1.0, 0.6, 0.2, 0.9) };
            draw_circle(center.x, center.y, 2.5, dot_color);
        }
    }
}

