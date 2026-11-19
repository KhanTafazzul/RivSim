//! Environmental Dynamics: River Current Vector Field, Turbulent Eddies, and Wind Gusts

use rand::Rng;

#[derive(Debug, Clone)]
pub struct VortexEddy {
    pub x: f32,
    pub y: f32,
    pub circulation: f32,
    pub core_radius: f32,
}

#[derive(Debug, Clone)]
pub struct EnvironmentEngine {
    // River Flow Field
    pub current_speed_mps: f32,
    pub current_dir_rad: f32,
    pub turbulence_intensity: f32,
    pub river_half_width: f32,
    pub eddies: Vec<VortexEddy>,

    // Wind Dynamics
    pub base_wind_speed_mps: f32,
    pub base_wind_dir_rad: f32,
    pub gust_probability: f32,
    pub current_wind_vx: f32,
    pub current_wind_vy: f32,
    gust_timer_s: f32,
}

impl EnvironmentEngine {
    pub fn new() -> Self {
        let mut eddies = Vec::new();
        // Seed initial turbulent eddies along river corridor
        for i in 0..5 {
            let offset_x = (i as f32 - 2.0) * 12.0;
            let offset_y = if i % 2 == 0 { 3.5 } else { -3.5 };
            eddies.push(VortexEddy {
                x: offset_x,
                y: offset_y,
                circulation: if i % 2 == 0 { 4.5 } else { -4.5 },
                core_radius: 2.5,
            });
        }

        Self {
            current_speed_mps: 0.85,
            current_dir_rad: 0.0f32.to_radians(), // Flowing in +Y downstream
            turbulence_intensity: 0.35,
            river_half_width: 12.0,
            eddies,
            base_wind_speed_mps: 3.5,
            base_wind_dir_rad: 45.0f32.to_radians(),
            gust_probability: 0.25,
            current_wind_vx: 2.5,
            current_wind_vy: 2.5,
            gust_timer_s: 0.0,
        }
    }

    /// Step wind gusts and drift eddies downstream
    pub fn step(&mut self, dt: f32) {
        let mut rng = rand::thread_rng();

        // 1. Drift turbulent eddies downstream along current direction
        let eddy_drift_x = -self.current_dir_rad.sin() * self.current_speed_mps * 0.7 * dt;
        let eddy_drift_y = self.current_dir_rad.cos() * self.current_speed_mps * 0.7 * dt;

        for eddy in &mut self.eddies {
            eddy.x += eddy_drift_x;
            eddy.y += eddy_drift_y;

            // Wrap around river bounds
            if eddy.y > 60.0 {
                eddy.y = -60.0;
                eddy.x = rng.gen_range(-self.river_half_width * 0.7..self.river_half_width * 0.7);
            } else if eddy.y < -60.0 {
                eddy.y = 60.0;
                eddy.x = rng.gen_range(-self.river_half_width * 0.7..self.river_half_width * 0.7);
            }
        }

        // 2. Wind Gust Dynamics (Ornstein-Uhlenbeck stochastic drift)
        self.gust_timer_s += dt;
        let base_vx = -self.base_wind_dir_rad.sin() * self.base_wind_speed_mps;
        let base_vy = self.base_wind_dir_rad.cos() * self.base_wind_speed_mps;

        let is_gusting = rng.gen_bool(self.gust_probability.clamp(0.0, 1.0) as f64);
        let gust_mult = if is_gusting { rng.gen_range(1.2..2.4) } else { 1.0 };

        let target_vx = base_vx * gust_mult + rng.gen_range(-0.5..0.5);
        let target_vy = base_vy * gust_mult + rng.gen_range(-0.5..0.5);

        // Smooth relaxation toward target wind
        let alpha = (dt * 1.5).min(1.0);
        self.current_wind_vx += (target_vx - self.current_wind_vx) * alpha;
        self.current_wind_vy += (target_vy - self.current_wind_vy) * alpha;
    }

    /// Calculate water flow velocity vector at world coordinate (x, y)
    pub fn get_water_flow_at(&self, x: f32, y: f32) -> (f32, f32) {
        // Parabolic boundary profile: current is fastest at river center (x = 0), slows near banks
        let bank_dist = (x.abs() / self.river_half_width).clamp(0.0, 1.0);
        let profile = (1.0 - bank_dist * bank_dist).max(0.15);
        let speed = self.current_speed_mps * profile;

        let base_vx = -self.current_dir_rad.sin() * speed;
        let base_vy = self.current_dir_rad.cos() * speed;

        // Sum vortex velocities from nearby turbulent eddies
        let mut eddy_vx = 0.0;
        let mut eddy_vy = 0.0;

        for eddy in &self.eddies {
            let dx = x - eddy.x;
            let dy = y - eddy.y;
            let r2 = dx * dx + dy * dy;
            let r0_2 = eddy.core_radius * eddy.core_radius;
            // Rankine/Lamb-Oseen vortex formulation: v_theta = Gamma / (2*pi * (r^2 + r0^2))
            let factor = (eddy.circulation * self.turbulence_intensity) / (6.28318 * (r2 + r0_2));
            eddy_vx += -dy * factor;
            eddy_vy += dx * factor;
        }

        (base_vx + eddy_vx, base_vy + eddy_vy)
    }

    /// Calculate wind velocity vector at world coordinate (x, y)
    pub fn get_wind_at(&self, _x: f32, _y: f32) -> (f32, f32) {
        (self.current_wind_vx, self.current_wind_vy)
    }

    /// Update water current parameters (called by MCP or visual controls)
    pub fn set_water_current(&mut self, speed_mps: f32, dir_deg: f32, turbulence: f32) {
        self.current_speed_mps = speed_mps.clamp(0.0, 5.0);
        self.current_dir_rad = dir_deg.to_radians();
        self.turbulence_intensity = turbulence.clamp(0.0, 2.0);
    }

    /// Update weather / wind parameters (called by MCP or visual controls)
    pub fn set_weather(&mut self, wind_speed_mps: f32, wind_dir_deg: f32, gust_prob: f32) {
        self.base_wind_speed_mps = wind_speed_mps.clamp(0.0, 25.0);
        self.base_wind_dir_rad = wind_dir_deg.to_radians();
        self.gust_probability = gust_prob.clamp(0.0, 1.0);
    }
}

