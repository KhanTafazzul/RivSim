//! Virtual NEO-6M GPS Module
//! Translates 2D world coordinates (x, y) into WGS84 Latitude/Longitude.
//! Injects 1.5m to 3m drift/noise radius at low speeds (satellite multipath bounce).
//! Formats standard NMEA sentences for UART1 stream.

use rand::Rng;
use crate::config::{ORIGIN_LATITUDE, ORIGIN_LONGITUDE, METERS_PER_DEGREE_LAT};

#[derive(Debug, Clone)]
pub struct GpsSensor {
    pub latitude: f64,
    pub longitude: f64,
    pub speed_mps: f32,
    pub speed_knots: f32,
    pub course_deg: f32,
    pub satellites: u8,
    pub hdop: f32,
    pub fault_spike: bool,
    noise_x: f32,
    noise_y: f32,
}

impl GpsSensor {
    pub fn new() -> Self {
        Self {
            latitude: ORIGIN_LATITUDE,
            longitude: ORIGIN_LONGITUDE,
            speed_mps: 0.0,
            speed_knots: 0.0,
            course_deg: 0.0,
            satellites: 9,
            hdop: 1.1,
            fault_spike: false,
            noise_x: 0.0,
            noise_y: 0.0,
        }
    }

    /// Update GPS state from rigid body position (x, y), speed, and heading
    pub fn update(&mut self, world_x: f32, world_y: f32, vx: f32, vy: f32, heading_deg: f32, dt: f32) {
        let speed = (vx * vx + vy * vy).sqrt();
        self.speed_mps = speed;
        self.speed_knots = speed * 1.94384;
        self.course_deg = (heading_deg % 360.0 + 360.0) % 360.0;

        let mut rng = rand::thread_rng();

        // Satellite bounce drift simulation at low speed (< 0.3 m/s)
        if speed < 0.3 {
            // Random-walk drift within 1.5m - 3.0m radius
            self.noise_x += rng.gen_range(-0.4..0.4) * dt;
            self.noise_y += rng.gen_range(-0.4..0.4) * dt;
            let drift_radius = (self.noise_x * self.noise_x + self.noise_y * self.noise_y).sqrt();
            if drift_radius > 2.5 {
                self.noise_x *= 2.5 / drift_radius;
                self.noise_y *= 2.5 / drift_radius;
            }
        } else {
            // Decay bounce drift when moving swiftly
            self.noise_x *= 0.85;
            self.noise_y *= 0.85;
        }

        let mut eff_x = world_x + self.noise_x;
        let mut eff_y = world_y + self.noise_y;

        // Injected GPS multipath spike fault
        if self.fault_spike {
            eff_x += 45.0;
            eff_y -= 38.0;
        }

        // Flat-earth WGS84 conversion
        let meters_per_deg_lon = METERS_PER_DEGREE_LAT * (ORIGIN_LATITUDE.to_radians()).cos();
        self.latitude = ORIGIN_LATITUDE + (eff_y as f64 / METERS_PER_DEGREE_LAT);
        self.longitude = ORIGIN_LONGITUDE + (eff_x as f64 / meters_per_deg_lon);
    }

    /// Generate an NMEA $GPRMC sentence string for the UART1 buffer
    pub fn format_nmea_gprmc(&self, sim_time_s: f32) -> String {
        let total_secs = sim_time_s as u32;
        let hh = (total_secs / 3600) % 24;
        let mm = (total_secs / 60) % 60;
        let ss = total_secs % 60;

        let lat_abs = self.latitude.abs();
        let lat_deg = lat_abs.floor() as u32;
        let lat_min = (lat_abs - lat_deg as f64) * 60.0;
        let lat_dir = if self.latitude >= 0.0 { 'N' } else { 'S' };

        let lon_abs = self.longitude.abs();
        let lon_deg = lon_abs.floor() as u32;
        let lon_min = (lon_abs - lon_deg as f64) * 60.0;
        let lon_dir = if self.longitude >= 0.0 { 'E' } else { 'W' };

        let payload = format!(
            "GPRMC,{:02}{:02}{:02}.00,A,{:02}{:07.4},{},{:03}{:07.4},{},{:.2},{:.1},191126,,,A",
            hh, mm, ss, lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir, self.speed_knots, self.course_deg
        );

        // NMEA XOR Checksum
        let mut checksum = 0u8;
        for b in payload.bytes() {
            checksum ^= b;
        }

        format!("${}*{:02X}\r\n", payload, checksum)
    }
}

