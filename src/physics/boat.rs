//! Boat Physical Model & Hydrodynamic/Aerodynamic Force Calculations
//! Implements compound catamaran rigid body properties and differential paddle wheel dynamics.

use rapier2d::prelude::*;
use crate::config::*;

#[derive(Debug, Clone)]
pub struct BoatModel {
    pub body_handle: RigidBodyHandle,
    pub left_paddle_pwm: i16,
    pub right_paddle_pwm: i16,
    pub conveyor_pwm: i16,
    pub left_wheel_angle: f32,
    pub right_wheel_angle: f32,
    pub conveyor_offset: f32,
    pub fault_motor_jam_left: bool,
    pub fault_motor_jam_right: bool,
}

impl BoatModel {
    pub fn new(bodies: &mut RigidBodySet, colliders: &mut ColliderSet, start_x: f32, start_y: f32, start_yaw_deg: f32) -> Self {
        let rad = start_yaw_deg.to_radians();
        
        // Rigid body with mass and shifted center of gravity
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(glam::vec2(start_x, start_y))
            .rotation(rad)
            .additional_mass(BOAT_MASS_KG)
            .linear_damping(0.4)
            .angular_damping(1.8)
            .build();

        let body_handle = bodies.insert(rigid_body);

        let half_pontoon_len = PONTOON_LENGTH_M * 0.5;
        let pontoon_radius = PONTOON_DIAMETER_M * 0.5;
        let half_spacing = PONTOON_SPACING_M * 0.5;

        // Left Pontoon Collider (Capsule along Y-axis)
        let left_pontoon = ColliderBuilder::capsule_y(half_pontoon_len - pontoon_radius, pontoon_radius)
            .translation(glam::vec2(-half_spacing, 0.0))
            .friction(0.2)
            .restitution(0.1)
            .build();
        colliders.insert_with_parent(left_pontoon, body_handle, bodies);

        // Right Pontoon Collider (Capsule along Y-axis)
        let right_pontoon = ColliderBuilder::capsule_y(half_pontoon_len - pontoon_radius, pontoon_radius)
            .translation(glam::vec2(half_spacing, 0.0))
            .friction(0.2)
            .restitution(0.1)
            .build();
        colliders.insert_with_parent(right_pontoon, body_handle, bodies);

        // Front Conveyor Belt Collider (Extends 1" wider on each side)
        let conveyor_half_w = CONVEYOR_WIDTH_M * 0.5;
        let conveyor_half_l = CONVEYOR_LENGTH_M * 0.5;
        let conveyor_y = half_pontoon_len + conveyor_half_l * 0.6;
        let conveyor_collider = ColliderBuilder::cuboid(conveyor_half_w, conveyor_half_l)
            .translation(glam::vec2(0.0, conveyor_y))
            .sensor(false)
            .build();
        colliders.insert_with_parent(conveyor_collider, body_handle, bodies);

        // Waste container & electronics enclosure
        let waste_collider = ColliderBuilder::cuboid(half_spacing * 0.8, 0.25)
            .translation(glam::vec2(COG_OFFSET_X, COG_OFFSET_Y))
            .build();
        colliders.insert_with_parent(waste_collider, body_handle, bodies);

        Self {
            body_handle,
            left_paddle_pwm: 0,
            right_paddle_pwm: 0,
            conveyor_pwm: 0,
            left_wheel_angle: 0.0,
            right_wheel_angle: 0.0,
            conveyor_offset: 0.0,
            fault_motor_jam_left: false,
            fault_motor_jam_right: false,
        }
    }

    /// Apply paddle wheel thrust and hydrodynamic/aerodynamic forces
    pub fn apply_forces(
        &mut self,
        bodies: &mut RigidBodySet,
        water_vx: f32,
        water_vy: f32,
        wind_vx: f32,
        wind_vy: f32,
        dt: f32,
    ) {
        let body = match bodies.get_mut(self.body_handle) {
            Some(b) => b,
            None => return,
        };

        let rot = body.rotation().angle();
        let forward = glam::vec2(-rot.sin(), rot.cos());
        let right = glam::vec2(rot.cos(), rot.sin());

        // 1. Actuator Thrust with Fault Jam Logic
        let eff_left_pwm = if self.fault_motor_jam_left { 0 } else { self.left_paddle_pwm };
        let eff_right_pwm = if self.fault_motor_jam_right { 0 } else { self.right_paddle_pwm };

        let f_left = (eff_left_pwm as f32) * MOTOR_THRUST_COEFF;
        let f_right = (eff_right_pwm as f32) * MOTOR_THRUST_COEFF;

        // Visual animation increments
        self.left_wheel_angle += (eff_left_pwm as f32) * dt * 0.08;
        self.right_wheel_angle += (eff_right_pwm as f32) * dt * 0.08;
        self.conveyor_offset += (self.conveyor_pwm as f32) * dt * 0.05;

        // Paddle attachment offsets in body frame
        let half_paddle_span = PADDLE_SPACING_M * 0.5;
        let paddle_y = -PONTOON_LENGTH_M * 0.35;

        // World thrust vectors
        let thrust_left_vec = forward * f_left;
        let thrust_right_vec = forward * f_right;

        // Apply forces at paddle positions (generates differential yaw moment)
        let pos_left_world = body.position().transform_point(glam::vec2(-half_paddle_span, paddle_y));
        let pos_right_world = body.position().transform_point(glam::vec2(half_paddle_span, paddle_y));

        body.add_force_at_point(thrust_left_vec, pos_left_world, true);
        body.add_force_at_point(thrust_right_vec, pos_right_world, true);

        // 2. Hydrodynamic Fluid Drag & Current Drift
        let boat_v = body.linvel();
        let water_v = glam::vec2(water_vx, water_vy);
        let rel_v_water = boat_v - water_v;

        // Project relative velocity into body longitudinal and lateral components
        let v_long = rel_v_water.dot(forward);
        let v_lat = rel_v_water.dot(right);

        // Pontoon cross-sectional areas with front conveyor extra drag plate
        let a_front = (PONTOON_DIAMETER_M * PONTOON_DIAMETER_M * 0.5 * 2.0) + (CONVEYOR_WIDTH_M * 0.08 * CD_CONVEYOR_WATER);
        let a_side = PONTOON_LENGTH_M * PONTOON_DIAMETER_M * 0.5 * 2.0;

        // Quadratic hydrodynamic drag
        let f_drag_long = -0.5 * WATER_DENSITY * CD_LONGITUDINAL * a_front * v_long.abs() * v_long;
        let f_drag_lat = -0.5 * WATER_DENSITY * CD_LATERAL * a_side * v_lat.abs() * v_lat;

        let total_hydro_force = (forward * f_drag_long) + (right * f_drag_lat);
        body.add_force(total_hydro_force, true);

        // Hydrodynamic rotational drag (water dampens rotation)
        let ang_vel = body.angvel();
        let hydro_torque = -0.5 * WATER_DENSITY * 0.8 * ang_vel.abs() * ang_vel;
        body.add_torque(hydro_torque, true);

        // 3. Aerodynamic Wind Drag (Waste Box & Conveyor Sail)
        let wind_v = glam::vec2(wind_vx, wind_vy);
        let rel_v_wind = wind_v - boat_v;
        let wind_speed_sq = rel_v_wind.length_squared();
        if wind_speed_sq > 0.01 {
            let wind_dir = rel_v_wind.normalize();
            let a_wind_front = CONVEYOR_WIDTH_M * 0.25;
            let f_aero = 0.5 * AIR_DENSITY * CD_AERO_WIND * a_wind_front * wind_speed_sq;
            let aero_force = wind_dir * f_aero;
            
            // Apply wind slightly forward of CoG (wind-catcher effect from conveyor)
            let wind_pos_world = body.position().transform_point(glam::vec2(0.0, 0.35));
            body.add_force_at_point(aero_force, wind_pos_world, true);
        }
    }
}
