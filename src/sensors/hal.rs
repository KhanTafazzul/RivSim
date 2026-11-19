//! ESP32 Hardware Abstraction Layer & JSON Telemetry Builder
//! Coordinates all virtual sensors and formats the official telemetry payload.

use serde::{Deserialize, Serialize};
use crate::sensors::{
    battery::BatteryModel, firmware::RustFirmwareController, gps::GpsSensor, imu::ImuSensor,
    ldr::LdrSensor, thermal::ThermalModel, turbidity::TurbiditySensor,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsTelemetry {
    pub lat: f64,
    pub lon: f64,
    pub speed_knots: f32,
    pub speed_mps: f32,
    pub hdg: f32,
    pub sats: u8,
    pub hdop: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuTelemetry {
    pub yaw: f32,
    pub gyro_z: f32,
    pub pitch: f32,
    pub roll: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryTelemetry {
    pub voltage: f32,
    pub percent: f32,
    pub current_draw_a: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaterQualityTelemetry {
    pub turbidity_ntu: f32,
    pub adc_raw: u16,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorsTelemetry {
    pub left_pwm: i16,
    pub right_pwm: i16,
    pub conveyor_pwm: i16,
    pub headlights: bool,
    pub driver_temp_c: f32,
    pub driver_throttled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemTelemetry {
    pub sim_time_s: f32,
    pub rssi: i32,
    pub link_connected: bool,
    pub failsafe_active: bool,
    pub firmware_state: String,
    pub target_heading_deg: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Esp32Telemetry {
    pub timestamp_ms: u64,
    pub gps: GpsTelemetry,
    pub imu: ImuTelemetry,
    pub battery: BatteryTelemetry,
    pub water_quality: WaterQualityTelemetry,
    pub actuators: ActuatorsTelemetry,
    pub system: SystemTelemetry,
}

#[derive(Debug, Clone)]
pub struct HardwareAbstractionLayer {
    pub gps: GpsSensor,
    pub imu: ImuSensor,
    pub battery: BatteryModel,
    pub turbidity: TurbiditySensor,
    pub ldr: LdrSensor,
    pub thermal: ThermalModel,
    pub firmware: RustFirmwareController,
}

impl HardwareAbstractionLayer {
    pub fn new() -> Self {
        Self {
            gps: GpsSensor::new(),
            imu: ImuSensor::new(),
            battery: BatteryModel::new(),
            turbidity: TurbiditySensor::new(),
            ldr: LdrSensor::new(),
            thermal: ThermalModel::new(),
            firmware: RustFirmwareController::new(),
        }
    }

    /// Build standard ESP32 telemetry JSON representation
    pub fn build_telemetry_json(
        &self,
        sim_time_s: f32,
        rssi: i32,
        link_connected: bool,
    ) -> Esp32Telemetry {
        Esp32Telemetry {
            timestamp_ms: (sim_time_s * 1000.0) as u64,
            gps: GpsTelemetry {
                lat: self.gps.latitude,
                lon: self.gps.longitude,
                speed_knots: self.gps.speed_knots,
                speed_mps: self.gps.speed_mps,
                hdg: self.gps.course_deg,
                sats: self.gps.satellites,
                hdop: self.gps.hdop,
            },
            imu: ImuTelemetry {
                yaw: self.imu.yaw_deg,
                gyro_z: self.imu.gyro_z_dps,
                pitch: self.imu.pitch_deg,
                roll: self.imu.roll_deg,
            },
            battery: BatteryTelemetry {
                voltage: self.battery.loaded_voltage,
                percent: self.battery.percentage,
                current_draw_a: self.battery.current_draw_a,
            },
            water_quality: WaterQualityTelemetry {
                turbidity_ntu: self.turbidity.ntu,
                adc_raw: self.turbidity.adc_raw,
            },
            actuators: ActuatorsTelemetry {
                left_pwm: self.firmware.commanded_left_pwm,
                right_pwm: self.firmware.commanded_right_pwm,
                conveyor_pwm: self.firmware.commanded_conveyor_pwm,
                headlights: self.ldr.headlights_active,
                driver_temp_c: self.thermal.temp_c,
                driver_throttled: self.thermal.throttled,
            },
            system: SystemTelemetry {
                sim_time_s,
                rssi,
                link_connected,
                failsafe_active: self.firmware.failsafe_active,
                firmware_state: format!("{:?}", self.firmware.state),
                target_heading_deg: self.firmware.target_heading_deg,
            },
        }
    }
}

