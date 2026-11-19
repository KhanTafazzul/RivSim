//! Rapier2D Physics World Engine & Simulation Orchestrator
//! Houses the rigid body world, river boundary colliders, boat, environment, and radio links.

use rapier2d::prelude::*;
use crate::physics::{boat::BoatModel, environment::EnvironmentEngine, radio::RadioSubsystem};

pub struct PhysicsEngine {
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub soft_bodies: SoftBodySet,
    pub ccd_solver: CCDSolver,
    pub integration_parameters: IntegrationParameters,

    pub boat: BoatModel,
    pub env: EnvironmentEngine,
    pub radio: RadioSubsystem,

    pub sim_time_s: f32,
    pub step_dt: f32,
}

impl PhysicsEngine {
    pub fn new(start_x: f32, start_y: f32, start_yaw_deg: f32) -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        // 1. Build Static River Banks (Collision bounds along x = -12.0 and x = +12.0)
        let river_half_width = 12.0;
        let bank_half_len = 100.0;

        // Left Bank
        let left_bank = RigidBodyBuilder::fixed()
            .translation(glam::vec2(-river_half_width - 1.0, 0.0))
            .build();
        let left_handle = bodies.insert(left_bank);
        let left_collider = ColliderBuilder::cuboid(1.0, bank_half_len)
            .restitution(0.2)
            .friction(0.5)
            .build();
        colliders.insert_with_parent(left_collider, left_handle, &mut bodies);

        // Right Bank
        let right_bank = RigidBodyBuilder::fixed()
            .translation(glam::vec2(river_half_width + 1.0, 0.0))
            .build();
        let right_handle = bodies.insert(right_bank);
        let right_collider = ColliderBuilder::cuboid(1.0, bank_half_len)
            .restitution(0.2)
            .friction(0.5)
            .build();
        colliders.insert_with_parent(right_collider, right_handle, &mut bodies);

        // 2. Instantiate Boat Compound Rigid Body
        let boat = BoatModel::new(&mut bodies, &mut colliders, start_x, start_y, start_yaw_deg);
        let env = EnvironmentEngine::new();
        let radio = RadioSubsystem::new();

        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 60.0;

        Self {
            bodies,
            colliders,
            pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            soft_bodies: SoftBodySet::new(),
            ccd_solver: CCDSolver::new(),
            integration_parameters,
            boat,
            env,
            radio,
            sim_time_s: 0.0,
            step_dt: 1.0 / 60.0,
        }
    }

    /// Step simulation forward by dt seconds
    pub fn step(&mut self, dt: f32) {
        self.step_dt = dt;
        self.integration_parameters.dt = dt;

        // 1. Query boat pose
        let (bx, by, _) = self.get_boat_pose();

        // 2. Step environmental dynamics & query fields at boat position
        self.env.step(dt);
        let (water_vx, water_vy) = self.env.get_water_flow_at(bx, by);
        let (wind_vx, wind_vy) = self.env.get_wind_at(bx, by);

        // 3. Update radio path loss & geofence status
        self.radio.update(bx, by);

        // 4. Apply motor thrust, fluid hydrodynamic drag, and aerodynamic wind drag
        self.boat.apply_forces(&mut self.bodies, water_vx, water_vy, wind_vx, wind_vy, dt);

        // 5. Advance Rapier2D rigid body solver
        let gravity = glam::Vec2::ZERO;
        let physics_hooks = ();
        let event_handler = ();

        self.pipeline.step(
            gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.soft_bodies,
            &mut self.ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        self.sim_time_s += dt;
    }

    /// Get current boat world position and yaw (x, y, yaw_rad)
    pub fn get_boat_pose(&self) -> (f32, f32, f32) {
        if let Some(body) = self.bodies.get(self.boat.body_handle) {
            let pos = body.position();
            (pos.translation.x, pos.translation.y, pos.rotation.angle())
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Get current boat linear velocities (vx, vy) and angular velocity (omega_z)
    pub fn get_boat_velocities(&self) -> (f32, f32, f32) {
        if let Some(body) = self.bodies.get(self.boat.body_handle) {
            let linvel = body.linvel();
            (linvel.x, linvel.y, body.angvel())
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Teleport boat to a specific coordinate and orientation
    pub fn teleport_boat(&mut self, x: f32, y: f32, yaw_deg: f32) {
        if let Some(body) = self.bodies.get_mut(self.boat.body_handle) {
            body.set_translation(glam::vec2(x, y), true);
            body.set_rotation(rapier2d::math::rotation_from_angle(yaw_deg.to_radians()), true);
            body.set_linvel(glam::Vec2::ZERO, true);
            body.set_angvel(0.0, true);
        }
    }
}
