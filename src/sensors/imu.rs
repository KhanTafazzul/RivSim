//! Virtual MPU-6050 6-Axis IMU
//! Reads absolute heading (yaw) and angular velocity from rigid body.
//! Injects sensor jitter and populates raw 16-bit register values for I2C 0x68.

use rand::Rng;

#[derive(Debug, Clone)]
pub struct ImuSensor {
    pub yaw_deg: f32,
    pub yaw_rad: f32,
    pub gyro_z_dps: f32,
    pub gyro_z_rad: f32,
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub raw_gyro_z: i16,
    pub raw_accel_x: i16,
    pub raw_accel_y: i16,
}

impl ImuSensor {
    pub fn new() -> Self {
        Self {
            yaw_deg: 0.0,
            yaw_rad: 0.0,
            gyro_z_dps: 0.0,
            gyro_z_rad: 0.0,
            pitch_deg: 0.0,
            roll_deg: 0.0,
            raw_gyro_z: 0,
            raw_accel_x: 0,
            raw_accel_y: 0,
        }
    }

    /// Update IMU from rigid body orientation and angular velocity
    pub fn update(&mut self, angle_rad: f32, angular_vel_rad: f32, sim_time_s: f32) {
        let mut rng = rand::thread_rng();

        // Angular velocity in deg/s with slight sensor jitter
        let jitter = rng.gen_range(-0.15..0.15);
        self.gyro_z_rad = angular_vel_rad;
        self.gyro_z_dps = angular_vel_rad.to_degrees() + jitter;

        // Yaw angle in degrees [0.0, 360.0)
        let deg = angle_rad.to_degrees();
        self.yaw_deg = (deg % 360.0 + 360.0) % 360.0;
        self.yaw_rad = angle_rad;

        // Simulated pontoon water wave rocking on pitch and roll
        let wave_phase = sim_time_s * 2.5;
        self.pitch_deg = (wave_phase.sin() * 0.8) + rng.gen_range(-0.1..0.1);
        self.roll_deg = ((wave_phase * 1.3).cos() * 1.2) + rng.gen_range(-0.1..0.1);

        // MPU-6050 raw registers (FS_SEL = 0: 131 LSB / deg/s)
        self.raw_gyro_z = (self.gyro_z_dps * 131.0).clamp(-32768.0, 32767.0) as i16;

        // Accelerometer 16384 LSB / g
        let ax = (self.pitch_deg.to_radians().sin() * 16384.0) as i16;
        let ay = (self.roll_deg.to_radians().sin() * 16384.0) as i16;
        self.raw_accel_x = ax;
        self.raw_accel_y = ay;
    }
}

