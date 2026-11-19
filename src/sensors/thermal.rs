//! BTS7960 Motor Driver NTC Thermistor & Thermal Throttling Model
//! Models heat generation from high motor currents and thermal dissipation.
//! Automatically clamps motor PWM to 150 when temperature exceeds 70.0°C.

#[derive(Debug, Clone)]
pub struct ThermalModel {
    pub temp_c: f32,
    pub ambient_temp_c: f32,
    pub adc_raw: u16,
    pub throttled: bool,
}

impl ThermalModel {
    pub fn new() -> Self {
        Self {
            temp_c: 28.0,
            ambient_temp_c: 26.0,
            adc_raw: 1400,
            throttled: false,
        }
    }

    /// Step the thermal integration forward by dt seconds with motor PWM load
    pub fn step(&mut self, left_pwm: i16, right_pwm: i16, dt: f32) {
        let load_left = (left_pwm.abs() as f32 / 255.0).powi(2);
        let load_right = (right_pwm.abs() as f32 / 255.0).powi(2);
        let total_load = load_left + load_right; // [0.0, 2.0]

        // Joule heating rate: at full 100% throttle, heats up to ~85°C
        let heat_gain = total_load * 3.2;

        // Dissipation rate proportional to temperature delta above ambient
        let heat_dissipation = (self.temp_c - self.ambient_temp_c) * 0.08;

        self.temp_c += (heat_gain - heat_dissipation) * dt;
        self.temp_c = self.temp_c.clamp(self.ambient_temp_c, 115.0);

        // Hardware safety threshold: clamp PWM if T > 70.0°C
        self.throttled = self.temp_c > 70.0;

        // NTC 10K thermistor curve with 10K pull-up on GPIO 39 (3.3V reference)
        // Higher temp -> lower resistance -> higher ADC reading
        let norm_temp = ((self.temp_c - 20.0) / 80.0).clamp(0.0, 1.0);
        self.adc_raw = (1200.0 + norm_temp * 2200.0) as u16;
    }

    /// Enforce safety clamping on motor PWM command
    pub fn clamp_pwm(&self, commanded_pwm: i16) -> i16 {
        if self.throttled {
            commanded_pwm.clamp(-150, 150)
        } else {
            commanded_pwm.clamp(-255, 255)
        }
    }
}
