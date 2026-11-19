//! Native Rust Fallback Firmware Controller
//! Runs onboard control loop if no C++ sketch is dynamically loaded.
//! Implements PID heading-hold, 3m parking anchor, 30s loiter, and GPS RTH.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareState {
    Manual = 0,
    HeadingHold = 1,
    ParkingAnchor = 2,
    Loiter = 3,
    ReturnToHome = 4,
}

#[derive(Debug, Clone)]
pub struct RustFirmwareController {
    pub state: FirmwareState,
    pub target_heading_deg: f32,
    pub home_x: f32,
    pub home_y: f32,
    pub anchor_x: Option<f32>,
    pub anchor_y: Option<f32>,

    // PID Gains
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_err: f32,
    pub prev_err: f32,

    // Hardware Calibration
    pub trim_left: f32,
    pub trim_right: f32,

    // Failsafe Timers
    pub signal_loss_start_s: Option<f32>,
    pub failsafe_active: bool,

    // Actuator Outputs
    pub commanded_left_pwm: i16,
    pub commanded_right_pwm: i16,
    pub commanded_conveyor_pwm: i16,
    pub commanded_headlights: bool,
}

impl RustFirmwareController {
    pub fn new() -> Self {
        Self {
            state: FirmwareState::HeadingHold,
            target_heading_deg: 0.0,
            home_x: 0.0,
            home_y: 0.0,
            anchor_x: None,
            anchor_y: None,
            kp: 2.4,
            ki: 0.05,
            kd: 0.8,
            integral_err: 0.0,
            prev_err: 0.0,
            trim_left: 1.0,
            trim_right: 0.96,
            signal_loss_start_s: None,
            failsafe_active: false,
            commanded_left_pwm: 0,
            commanded_right_pwm: 0,
            commanded_conveyor_pwm: 0,
            commanded_headlights: false,
        }
    }

    /// Update firmware state machine and compute motor outputs
    pub fn update(
        &mut self,
        boat_x: f32,
        boat_y: f32,
        current_yaw_deg: f32,
        wifi_connected: bool,
        thermal_throttled: bool,
        sim_time_s: f32,
        dt: f32,
    ) {
        // 1. Failsafe State Machine Transitions
        if !wifi_connected {
            if self.signal_loss_start_s.is_none() {
                self.signal_loss_start_s = Some(sim_time_s);
                self.failsafe_active = true;
                if self.state != FirmwareState::ReturnToHome {
                    self.state = FirmwareState::Loiter;
                }
            } else if let Some(loss_time) = self.signal_loss_start_s {
                // If signal severed for > 30 seconds, transition from Loiter to RTH
                if sim_time_s - loss_time > 30.0 && self.state == FirmwareState::Loiter {
                    self.state = FirmwareState::ReturnToHome;
                }
            }
        } else {
            // Signal healthy: clear loss timer unless already actively executing RTH
            self.signal_loss_start_s = None;
            self.failsafe_active = false;
        }

        let max_pwm_allowed: i16 = if thermal_throttled { 150 } else { 255 };
        let mut base_throttle: f32 = 0.0;
        let mut desired_heading = self.target_heading_deg;

        // 2. Behavioral Modes
        match self.state {
            FirmwareState::Manual => {
                // Handled directly by manual command input; clamp to thermal limits
                self.commanded_left_pwm = self.commanded_left_pwm.clamp(-max_pwm_allowed, max_pwm_allowed);
                self.commanded_right_pwm = self.commanded_right_pwm.clamp(-max_pwm_allowed, max_pwm_allowed);
                return;
            }
            FirmwareState::HeadingHold => {
                // Maintain heading at current manual base throttle or idle
                base_throttle = (self.commanded_left_pwm.abs().max(self.commanded_right_pwm.abs()) as f32).min(200.0);
                if base_throttle < 40.0 {
                    base_throttle = 0.0; // Deadband at rest
                }
            }
            FirmwareState::ParkingAnchor => {
                // Record anchor position on first entry
                if self.anchor_x.is_none() {
                    self.anchor_x = Some(boat_x);
                    self.anchor_y = Some(boat_y);
                }
                if let (Some(ax), Some(ay)) = (self.anchor_x, self.anchor_y) {
                    let dx = ax - boat_x;
                    let dy = ay - boat_y;
                    let drift = (dx * dx + dy * dy).sqrt();
                    if drift > 3.0 {
                        // Drift exceeded 3m parking anchor threshold: drive back to anchor
                        desired_heading = dy.atan2(dx).to_degrees();
                        base_throttle = 160.0;
                    } else {
                        base_throttle = 0.0;
                    }
                }
            }
            FirmwareState::Loiter => {
                // 30s Loiter: Hold low forward throttle heading into current
                base_throttle = 90.0;
            }
            FirmwareState::ReturnToHome => {
                let dx = self.home_x - boat_x;
                let dy = self.home_y - boat_y;
                let dist = (dx * dx + dy * dy).sqrt();
                if dist > 1.5 {
                    desired_heading = dy.atan2(dx).to_degrees();
                    base_throttle = 190.0;
                } else {
                    base_throttle = 0.0; // Reached home!
                }
            }
        }

        self.target_heading_deg = desired_heading;

        // 3. PID Heading-Hold Computation
        let mut err = desired_heading - current_yaw_deg;
        while err > 180.0 { err -= 360.0; }
        while err < -180.0 { err += 360.0; }

        let step_dt = if dt <= 0.001 || dt > 0.5 { 0.02 } else { dt };
        self.integral_err += err * step_dt;
        // Anti-windup clamping
        self.integral_err = self.integral_err.clamp(-45.0, 45.0);

        let derivative = (err - self.prev_err) / step_dt;
        self.prev_err = err;

        let steering = (self.kp * err) + (self.ki * self.integral_err) + (self.kd * derivative);
        let steering = steering.clamp(-120.0, 120.0);

        let left_unclamped = (base_throttle - steering) * self.trim_left;
        let right_unclamped = (base_throttle + steering) * self.trim_right;

        self.commanded_left_pwm = (left_unclamped as i16).clamp(-max_pwm_allowed, max_pwm_allowed);
        self.commanded_right_pwm = (right_unclamped as i16).clamp(-max_pwm_allowed, max_pwm_allowed);
    }
}

