//! Shared FFI Bridge State between Rust Simulator and ESP32 C++ Firmware
//! Memory layout must strictly match mock_hal/hal_bridge.h

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct HalBridgeState {
    pub sim_micros: u64,

    // Analog Inputs (12-bit ADC: 0 - 4095)
    pub adc_turbidity_pin34: u16,
    pub adc_battery_pin35: u16,
    pub adc_ldr_pin36: u16,
    pub adc_ntc_pin39: u16,

    // I2C 0x68 (MPU-6050)
    pub mpu6050_gyro_z_raw: i16,
    pub mpu6050_accel_x_raw: i16,
    pub mpu6050_accel_y_raw: i16,

    // UART1 GPS NMEA Buffer (GPIO 16/17)
    pub uart1_nmea_buf: [u8; 128],
    pub uart1_nmea_len: u8,
    pub uart1_read_pos: u8,

    // Actuator & Output Pins (Controlled by Firmware)
    pub pwm_motor_left: i16,
    pub pwm_motor_right: i16,
    pub pwm_conveyor: i16,
    pub gpio_headlights: u8,

    // Firmware Status & Failsafe Flags
    pub failsafe_active: u8,
    pub firmware_state: u8,
    pub thermal_throttled: u8,
    pub target_heading_deg: f32,
}

impl HalBridgeState {
    pub fn new() -> Self {
        Self {
            sim_micros: 0,
            adc_turbidity_pin34: 2800,
            adc_battery_pin35: 3120,
            adc_ldr_pin36: 2800,
            adc_ntc_pin39: 1400,
            mpu6050_gyro_z_raw: 0,
            mpu6050_accel_x_raw: 0,
            mpu6050_accel_y_raw: 0,
            uart1_nmea_buf: [0; 128],
            uart1_nmea_len: 0,
            uart1_read_pos: 0,
            pwm_motor_left: 0,
            pwm_motor_right: 0,
            pwm_conveyor: 0,
            gpio_headlights: 0,
            failsafe_active: 0,
            firmware_state: 1, // HeadingHold
            thermal_throttled: 0,
            target_heading_deg: 0.0,
        }
    }

    /// Load simulated sensor data into C bridge before firmware loop runs
    pub fn populate_inputs(
        &mut self,
        sim_time_s: f32,
        adc_turbidity: u16,
        adc_battery: u16,
        adc_ldr: u16,
        adc_ntc: u16,
        gyro_z: i16,
        accel_x: i16,
        accel_y: i16,
        nmea_sentence: &str,
        failsafe_engaged: bool,
    ) {
        self.sim_micros = (sim_time_s * 1_000_000.0) as u64;
        self.adc_turbidity_pin34 = adc_turbidity;
        self.adc_battery_pin35 = adc_battery;
        self.adc_ldr_pin36 = adc_ldr;
        self.adc_ntc_pin39 = adc_ntc;

        self.mpu6050_gyro_z_raw = gyro_z;
        self.mpu6050_accel_x_raw = accel_x;
        self.mpu6050_accel_y_raw = accel_y;

        self.failsafe_active = if failsafe_engaged { 1 } else { 0 };

        // Copy NMEA sentence to UART1 buffer
        let bytes = nmea_sentence.as_bytes();
        let copy_len = bytes.len().min(127);
        self.uart1_nmea_buf[..copy_len].copy_from_slice(&bytes[..copy_len]);
        self.uart1_nmea_buf[copy_len] = 0;
        self.uart1_nmea_len = copy_len as u8;
        self.uart1_read_pos = 0;
    }
}

