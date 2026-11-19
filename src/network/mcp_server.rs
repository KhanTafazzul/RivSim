//! Native Embedded MCP (Model Context Protocol) Server (127.0.0.1:8000/mcp)
//! Built on rmcp + axum + schemars to expose 9 AI test orchestration tools.

use std::sync::Arc;
use axum::{
    routing::{get, post},
    Router, Json, extract::State,
};
use tokio::net::TcpListener;
use tokio::sync::RwLock;
use serde_json::{json, Value};
use rmcp::{
    ServerHandler, tool, tool_handler, tool_router,
    handler::server::router::tool::ToolRouter,
    handler::server::wrapper::Parameters,
};

use crate::config::MCP_PORT;
use crate::dataset::{run_episode, EpisodeConfig};
use crate::headless::HeadlessBatchRunner;
use crate::network::protocol::*;
use crate::network::ws_server::SimulationCommand;
use crate::sensors::hal::Esp32Telemetry;

#[derive(Clone)]
pub struct McpServerState {
    pub cmd_sender: crossbeam_channel::Sender<SimulationCommand>,
    pub latest_telemetry: Arc<RwLock<Option<Esp32Telemetry>>>,
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for RivSimMcpService {}

#[derive(Clone)]
pub struct RivSimMcpService {
    tool_router: ToolRouter<Self>,
    pub state: McpServerState,
}

impl RivSimMcpService {
    pub fn new(state: McpServerState) -> Self {
        Self {
            tool_router: Self::tool_router(),
            state,
        }
    }
}

#[tool_router(router = tool_router)]
impl RivSimMcpService {
    /// Upload and dynamically compile an ESP32 C++ firmware sketch into the simulation
    #[tool(name = "upload_esp32_firmware", description = "Upload and dynamically compile an ESP32 C++ sketch (.cpp/.ino) via cl.exe/g++ and hot-load it.")]
    pub async fn upload_esp32_firmware(&self, params: Parameters<UploadFirmwareParams>) -> String {
        let p = params.0;
        if let Some(code) = p.inline_code_optional {
            let temp_path = std::env::temp_dir().join("uploaded_sketch.cpp");
            let _ = std::fs::write(&temp_path, code);
            let _ = self.state.cmd_sender.send(SimulationCommand::UploadFirmware {
                source_path: temp_path.to_string_lossy().to_string(),
            });
            format!("Inline sketch written and compilation queued from {}", temp_path.display())
        } else {
            let _ = self.state.cmd_sender.send(SimulationCommand::UploadFirmware {
                source_path: p.source_path.clone(),
            });
            format!("Firmware compilation queued for {}", p.source_path)
        }
    }

    /// Instantly teleport the boat to a target position, yaw, and battery percentage
    #[tool(name = "teleport_boat", description = "Teleport the boat to specific world coordinates (x, y), yaw heading in degrees, and set battery percentage.")]
    pub async fn teleport_boat(&self, params: Parameters<TeleportBoatParams>) -> String {
        let p = params.0;
        let _ = self.state.cmd_sender.send(SimulationCommand::TeleportBoat {
            x: p.x,
            y: p.y,
            yaw_deg: p.yaw_deg,
            battery_pct: p.battery_pct,
        });
        format!("Teleported boat to ({:.2}, {:.2}), yaw: {:.1}°, battery: {:.1}%", p.x, p.y, p.yaw_deg, p.battery_pct)
    }

    /// Set target home waypoint coordinates for Auto-Return-to-Home
    #[tool(name = "set_home_coordinates", description = "Set home waypoint coordinates (x, y) for GPS Return-to-Home navigation.")]
    pub async fn set_home_coordinates(&self, params: Parameters<SetHomeCoordinatesParams>) -> String {
        let p = params.0;
        let _ = self.state.cmd_sender.send(SimulationCommand::SetHomeCoordinates { x: p.x, y: p.y });
        format!("Home coordinates set to ({:.2}, {:.2})", p.x, p.y)
    }

    /// Modify the river current vector field speed, direction, and turbulence
    #[tool(name = "set_water_current", description = "Set river current speed (m/s), flow direction (deg), and turbulence intensity.")]
    pub async fn set_water_current(&self, params: Parameters<SetWaterCurrentParams>) -> String {
        let p = params.0;
        let _ = self.state.cmd_sender.send(SimulationCommand::SetWaterCurrent {
            speed_mps: p.speed_mps,
            dir_deg: p.direction_deg,
            turbulence: p.turbulence,
        });
        format!("River flow updated: {:.2} m/s @ {:.1}°, turbulence: {:.2}", p.speed_mps, p.direction_deg, p.turbulence)
    }

    /// Set atmospheric wind speed, direction, and gust probability
    #[tool(name = "set_weather", description = "Set wind speed (m/s), direction (deg), and gust probability [0.0 - 1.0].")]
    pub async fn set_weather(&self, params: Parameters<SetWeatherParams>) -> String {
        let p = params.0;
        let _ = self.state.cmd_sender.send(SimulationCommand::SetWeather {
            wind_speed_mps: p.wind_speed_mps,
            wind_dir_deg: p.wind_dir_deg,
            gust_prob: p.gust_probability,
        });
        format!("Weather updated: wind {:.2} m/s @ {:.1}°, gust prob: {:.2}", p.wind_speed_mps, p.wind_dir_deg, p.gust_probability)
    }

    /// Inject a simulated hardware/network fault (motor jam, GPS multipath spike, or Wi-Fi drop)
    #[tool(name = "inject_fault", description = "Inject hardware or RF fault: motor_jam_left, motor_jam_right, gps_spike, wifi_drop with duration in seconds.")]
    pub async fn inject_fault(&self, params: Parameters<InjectFaultParams>) -> String {
        let p = params.0;
        let _ = self.state.cmd_sender.send(SimulationCommand::InjectFault {
            fault_type: p.fault_type.clone(),
            _duration_sec: p.duration_sec,
        });
        format!("Injected fault '{}' for {:.1} seconds", p.fault_type, p.duration_sec)
    }

    /// Get instantaneous snapshot of full boat telemetry and sensors
    #[tool(name = "get_telemetry_snapshot", description = "Retrieve live ESP32 telemetry JSON snapshot including GPS, IMU, battery, turbidity, and failsafes.")]
    pub async fn get_telemetry_snapshot(&self) -> String {
        let opt_tel = self.state.latest_telemetry.read().await.clone();
        if let Some(t) = opt_tel {
            serde_json::to_string_pretty(&t).unwrap_or_else(|_| "{}".to_string())
        } else {
            "{\"status\": \"initializing\"}".to_string()
        }
    }

    /// Run a single complete return-to-home simulation episode and log steps to CSV
    #[tool(name = "run_return_episode", description = "Run a complete single Return-to-Home episode and append 28-column trajectory rows to CSV.")]
    pub async fn run_return_episode(&self, params: Parameters<RunReturnEpisodeParams>) -> String {
        let p = params.0;
        let cfg = EpisodeConfig {
            episode_id: 1,
            start_x: p.start_x,
            start_y: p.start_y,
            home_x: p.home_x,
            home_y: p.home_y,
            starting_battery_pct: p.battery_pct,
            water_speed: p.water_speed,
            water_dir_deg: p.water_dir,
            turbulence: p.turbulence,
            wind_speed: p.wind_speed,
            wind_dir_deg: p.wind_dir,
            exploration_noise: p.exploration_noise,
            max_sim_sec: p.max_sim_sec,
        };

        let file = match std::fs::OpenOptions::new().create(true).append(true).open(&p.csv_path) {
            Ok(f) => f,
            Err(e) => return format!("Failed to open CSV file {}: {}", p.csv_path, e),
        };
        let mut writer = std::io::BufWriter::new(file);
        let (steps, reached, dead, breach) = run_episode(&cfg, &mut writer, 50.0);

        format!(
            "Episode completed: {} steps. Reached home: {}, Battery dead: {}, Geofence breach: {}. Logged to {}",
            steps, reached, dead, breach, p.csv_path
        )
    }

    /// Trigger multi-core parallel headless batch generation of RL return dataset
    #[tool(name = "run_headless_return_dataset", description = "Trigger 4-core parallel Rayon headless batch simulation generating 1M-5M rows of return dataset.")]
    pub async fn run_headless_return_dataset(&self, params: Parameters<RunHeadlessReturnDatasetParams>) -> String {
        let p = params.0;
        let (rows, success, secs) = HeadlessBatchRunner::run_batch(
            p.num_episodes,
            p.target_rows,
            p.output_csv_path.clone(),
            p.enable_exploration,
        );

        format!(
            "Headless batch complete: {} rows generated across {} episodes ({} successful returns) in {:.2}s -> {}",
            rows, p.num_episodes, success, secs, p.output_csv_path
        )
    }
}

// HTTP REST / JSON-RPC fallback handlers for AI tools
async fn list_tools_handler() -> Json<Value> {
    Json(json!({
        "tools": [
            { "name": "upload_esp32_firmware", "description": "Upload and compile C++ sketch" },
            { "name": "teleport_boat", "description": "Teleport boat to coordinate and set battery" },
            { "name": "set_home_coordinates", "description": "Set home coordinates" },
            { "name": "set_water_current", "description": "Set river current speed, angle, turbulence" },
            { "name": "set_weather", "description": "Set wind speed, angle, gusts" },
            { "name": "inject_fault", "description": "Inject motor jam, GPS spike, or Wi-Fi drop" },
            { "name": "get_telemetry_snapshot", "description": "Get current telemetry JSON" },
            { "name": "run_return_episode", "description": "Run single return episode to CSV" },
            { "name": "run_headless_return_dataset", "description": "Run parallel Rayon batch dataset generator" }
        ]
    }))
}

async fn call_tool_handler(
    State(service): State<Arc<RivSimMcpService>>,
    Json(payload): Json<Value>,
) -> Json<Value> {
    let tool_name = payload["name"].as_str().unwrap_or("");
    let arguments = &payload["arguments"];

    let response_text = match tool_name {
        "teleport_boat" => {
            if let Ok(p) = serde_json::from_value::<TeleportBoatParams>(arguments.clone()) {
                service.teleport_boat(Parameters(p)).await
            } else {
                "Invalid parameters for teleport_boat".to_string()
            }
        }
        "set_water_current" => {
            if let Ok(p) = serde_json::from_value::<SetWaterCurrentParams>(arguments.clone()) {
                service.set_water_current(Parameters(p)).await
            } else {
                "Invalid parameters for set_water_current".to_string()
            }
        }
        "set_weather" => {
            if let Ok(p) = serde_json::from_value::<SetWeatherParams>(arguments.clone()) {
                service.set_weather(Parameters(p)).await
            } else {
                "Invalid parameters for set_weather".to_string()
            }
        }
        "inject_fault" => {
            if let Ok(p) = serde_json::from_value::<InjectFaultParams>(arguments.clone()) {
                service.inject_fault(Parameters(p)).await
            } else {
                "Invalid parameters for inject_fault".to_string()
            }
        }
        "set_home_coordinates" => {
            if let Ok(p) = serde_json::from_value::<SetHomeCoordinatesParams>(arguments.clone()) {
                service.set_home_coordinates(Parameters(p)).await
            } else {
                "Invalid parameters for set_home_coordinates".to_string()
            }
        }
        "upload_esp32_firmware" => {
            if let Ok(p) = serde_json::from_value::<UploadFirmwareParams>(arguments.clone()) {
                service.upload_esp32_firmware(Parameters(p)).await
            } else {
                "Invalid parameters for upload_esp32_firmware".to_string()
            }
        }
        "get_telemetry_snapshot" => service.get_telemetry_snapshot().await,
        "run_return_episode" => {
            if let Ok(p) = serde_json::from_value::<RunReturnEpisodeParams>(arguments.clone()) {
                service.run_return_episode(Parameters(p)).await
            } else {
                "Invalid parameters for run_return_episode".to_string()
            }
        }
        "run_headless_return_dataset" => {
            if let Ok(p) = serde_json::from_value::<RunHeadlessReturnDatasetParams>(arguments.clone()) {
                service.run_headless_return_dataset(Parameters(p)).await
            } else {
                "Invalid parameters for run_headless_return_dataset".to_string()
            }
        }
        _ => format!("Unknown MCP tool: {}", tool_name),
    };

    Json(json!({ "content": [{ "type": "text", "text": response_text }] }))
}

pub async fn start_mcp_server(
    state: McpServerState,
    mut shutdown_rx: tokio::sync::broadcast::Receiver<()>,
) {
    let service = Arc::new(RivSimMcpService::new(state));
    let app = Router::new()
        .route("/mcp", get(list_tools_handler))
        .route("/mcp/tools", get(list_tools_handler))
        .route("/mcp/call", post(call_tool_handler))
        .route("/tools/call", post(call_tool_handler))
        .with_state(service);

    let addr = format!("127.0.0.1:{}", MCP_PORT);
    let listener = match TcpListener::bind(&addr).await {
        Ok(l) => {
            println!("[RivSim MCP] AI orchestration server listening on http://{}/mcp", addr);
            l
        }
        Err(e) => {
            eprintln!("[RivSim MCP] Failed to bind to {}: {}", addr, e);
            return;
        }
    };

    tokio::select! {
        _ = axum::serve(listener, app) => {}
        _ = shutdown_rx.recv() => {}
    }
}
