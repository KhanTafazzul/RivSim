//! LDR Ambient Light Sensor & Virtual Headlights Controller
//! Simulates ambient light levels (Lux) and ADC readings on GPIO 36.
//! Triggers virtual headlights when ambient lux drops below 80 Lux.

#[derive(Debug, Clone)]
pub struct LdrSensor {
    pub lux: f32,
    pub adc_raw: u16,
    pub headlights_active: bool,
    pub auto_mode: bool,
}

impl LdrSensor {
    pub fn new() -> Self {
        Self {
            lux: 350.0,
            adc_raw: 2800,
            headlights_active: false,
            auto_mode: true,
        }
    }

    /// Update ambient light based on elapsed simulation time or manual daylight parameter
    pub fn update(&mut self, sim_time_s: f32, bridge_override: Option<bool>) {
        // Simulates a smooth diurnal cycle: 0 -> day (500 lux), sunset/night (30 lux)
        // Cycle period: 300 seconds (5 minutes) for simulation demonstration
        let cycle_phase = (sim_time_s % 300.0) / 300.0 * std::f32::consts::TAU;
        let sunlight = (cycle_phase.sin() + 1.0) * 0.5; // [0.0, 1.0]
        self.lux = 20.0 + sunlight * 480.0; // 20 Lux (dusk) to 500 Lux (day)

        // Phototransistor / LDR voltage divider on GPIO 36:
        // Dark -> high resistance -> ADC ~800; Bright -> low resistance -> ADC ~3500
        let norm = (self.lux / 500.0).clamp(0.0, 1.0);
        self.adc_raw = (600.0 + norm * 2900.0) as u16;

        // Auto headlights logic (threshold 80 Lux or ADC < 1200)
        if let Some(cmd_val) = bridge_override {
            self.headlights_active = cmd_val;
        } else if self.auto_mode {
            self.headlights_active = self.lux < 80.0;
        }
    }
}

