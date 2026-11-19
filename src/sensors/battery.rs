//! Virtual Battery Subsystem (3S Li-ion / LiFePO4)
//! Simulates realistic discharge curves, current consumption, and load voltage sag.

#[derive(Debug, Clone)]
pub struct BatteryModel {
    pub capacity_mah: f32,
    pub remaining_mah: f32,
    pub open_circuit_voltage: f32,
    pub loaded_voltage: f32,
    pub current_draw_a: f32,
    pub percentage: f32,
    pub adc_raw: u16,
    pub internal_resistance_ohms: f32,
}

impl BatteryModel {
    pub fn new() -> Self {
        let capacity_mah = 5000.0;
        Self {
            capacity_mah,
            remaining_mah: capacity_mah,
            open_circuit_voltage: 12.6,
            loaded_voltage: 12.6,
            current_draw_a: 0.15, // Quiescent draw (ESP32 + sensors)
            percentage: 100.0,
            adc_raw: 3120,
            internal_resistance_ohms: 0.08,
        }
    }

    /// Reset battery to a specific starting percentage [0.0, 100.0]
    pub fn set_percentage(&mut self, pct: f32) {
        self.percentage = pct.clamp(0.0, 100.0);
        self.remaining_mah = self.capacity_mah * (self.percentage / 100.0);
        self.update_voltage();
    }

    /// Step battery state with elapsed time dt and motor/conveyor/headlight loads
    pub fn step(&mut self, left_pwm: i16, right_pwm: i16, conveyor_pwm: i16, headlights: bool, dt: f32) {
        let base_current = 0.15; // ESP32 + sensors
        let motor_left_current = (left_pwm.abs() as f32 / 255.0) * 4.2; // Up to 4.2A per paddle
        let motor_right_current = (right_pwm.abs() as f32 / 255.0) * 4.2;
        let conveyor_current = (conveyor_pwm.abs() as f32 / 255.0) * 2.0;
        let headlight_current = if headlights { 0.45 } else { 0.0 };

        self.current_draw_a = base_current + motor_left_current + motor_right_current + conveyor_current + headlight_current;

        // Subtract mAh: Amps * (dt / 3600 hours) * 1000 mA/A
        let drained_mah = self.current_draw_a * (dt / 3.6);
        self.remaining_mah = (self.remaining_mah - drained_mah).max(0.0);
        self.percentage = ((self.remaining_mah / self.capacity_mah) * 100.0).clamp(0.0, 100.0);

        self.update_voltage();
    }

    fn update_voltage(&mut self) {
        // 3S Li-ion characteristic curve (12.6V fully charged -> 9.9V cutoff)
        let soc = self.percentage / 100.0;
        self.open_circuit_voltage = if soc > 0.85 {
            12.2 + (soc - 0.85) * (12.6 - 12.2) / 0.15
        } else if soc > 0.20 {
            11.1 + (soc - 0.20) * (12.2 - 11.1) / 0.65
        } else {
            9.9 + soc * (11.1 - 9.9) / 0.20
        };

        // Voltage sag under active load: V_loaded = V_oc - I * R_int
        self.loaded_voltage = (self.open_circuit_voltage - self.current_draw_a * self.internal_resistance_ohms).max(9.0);

        // ADC reading: 4:1 voltage divider (12.6V -> ~3.15V) to 3.3V 12-bit ADC
        let pin_v = self.loaded_voltage * (3.3 / 13.2);
        self.adc_raw = ((pin_v / 3.3) * 4095.0).clamp(0.0, 4095.0) as u16;
    }

    pub fn is_depleted(&self) -> bool {
        self.percentage <= 1.0 || self.loaded_voltage < 9.6
    }
}

