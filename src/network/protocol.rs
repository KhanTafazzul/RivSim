//! Network Protocol Schemas for Flutter SITL WebSocket & MCP Server

use serde::{Deserialize, Serialize};
use schemars::JsonSchema;

// --- Flutter WebSocket Inbound Commands ---

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd")]
pub enum FlutterCommand {
    #[serde(rename = "manual_drive")]
    ManualDrive {
        left: i16,
        right: i16,
    },
    #[serde(rename = "conveyor")]
    Conveyor {
        speed: i16,
    },
    #[serde(rename = "set_anchor")]
    SetAnchor {
        active: bool,
    },
    #[serde(rename = "set_heading")]
    SetHeading {
        deg: f32,
    },
    #[serde(rename = "auto_return")]
    AutoReturn,
    #[serde(rename = "headlights")]
    Headlights {
        active: bool,
    },
}

// --- MCP Tool Parameter Schemas (with JSON Schema support) ---

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct UploadFirmwareParams {
    pub source_path: String,
    pub inline_code_optional: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct TeleportBoatParams {
    pub x: f32,
    pub y: f32,
    pub yaw_deg: f32,
    pub battery_pct: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct SetHomeCoordinatesParams {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct SetWaterCurrentParams {
    pub speed_mps: f32,
    pub direction_deg: f32,
    pub turbulence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct SetWeatherParams {
    pub wind_speed_mps: f32,
    pub wind_dir_deg: f32,
    pub gust_probability: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct InjectFaultParams {
    pub fault_type: String, // motor_jam_left, motor_jam_right, gps_spike, wifi_drop
    pub duration_sec: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct RunReturnEpisodeParams {
    pub start_x: f32,
    pub start_y: f32,
    pub home_x: f32,
    pub home_y: f32,
    pub battery_pct: f32,
    pub water_speed: f32,
    pub water_dir: f32,
    pub turbulence: f32,
    pub wind_speed: f32,
    pub wind_dir: f32,
    pub exploration_noise: f32,
    pub max_sim_sec: f32,
    pub csv_path: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct RunHeadlessReturnDatasetParams {
    pub num_episodes: u32,
    pub target_rows: u32,
    pub output_csv_path: String,
    pub enable_exploration: bool,
}
