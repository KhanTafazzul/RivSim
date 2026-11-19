//! Spatial River Turbidity Field & Virtual Sensor Model
//! Simulates water clarity / turbidity (NTU) across the river surface,
//! taking into account spatial sediment runoff and water current disturbance.
//! Used for testing the Flutter Water Quality Heatmap visualization.

#[derive(Debug, Clone)]
pub struct TurbiditySensor {
    pub ntu: f32,
    pub adc_raw: u16,
}

impl TurbiditySensor {
    pub fn new() -> Self {
        Self {
            ntu: 15.0,
            adc_raw: 2800,
        }
    }

    /// Sample spatial turbidity at boat coordinates (x, y) with current speed and turbulence
    pub fn sample(&mut self, x: f32, y: f32, current_speed: f32, turbulence: f32) {
        // Base river clarity: cleaner upstream (x < 0), higher sediment near banks (|y| > 6.0)
        let bank_proximity = (y.abs() / 8.0).min(1.5);
        let base_ntu = 12.0 + 8.0 * (bank_proximity * bank_proximity);
        
        // Dynamic sediment agitation caused by rapid river current & turbulence
        let disturbance = current_speed * 4.5 + turbulence * 6.0;
        
        // Spatial sediment hotspots (e.g., stormwater runoff pipe at x = 15.0, y = -7.0)
        let dx = x - 15.0;
        let dy = y - (-7.0);
        let dist_runoff = (dx * dx + dy * dy).sqrt();
        let runoff_ntu = if dist_runoff < 8.0 {
            (8.0 - dist_runoff) * 12.0
        } else {
            0.0
        };

        let calculated_ntu = (base_ntu + disturbance + runoff_ntu).clamp(5.0, 350.0);
        self.ntu = calculated_ntu;

        // ESP32 ADC: Inverted characteristic for optical turbidity sensors
        // Clean water (0 NTU) -> ~4.2V (ADC ~3440)
        // Highly turbid water (300 NTU) -> ~2.5V (ADC ~2050)
        let voltage = (4.2 - (calculated_ntu / 300.0) * 1.7).clamp(1.5, 4.2);
        self.adc_raw = ((voltage / 3.3) * 4095.0).clamp(0.0, 4095.0) as u16;
    }
}

