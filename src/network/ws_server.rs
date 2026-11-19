//! Tokio WebSocket SITL Server (127.0.0.1:81)
//! Ingests Flutter commands and broadcasts 1 Hz ESP32 telemetry JSON.
//! Simulates RF packet loss / disconnection and provides optional AES-256-CBC encryption.

use std::sync::Arc;
use tokio::net::TcpListener;
use tokio::sync::{broadcast, RwLock};
use tokio_tungstenite::accept_async;
use futures_util::{StreamExt, SinkExt};
use cbc::cipher::{BlockEncryptMut, BlockDecryptMut, KeyIvInit, block_padding::Pkcs7};
use aes::Aes256;

use crate::config::{WS_PORT, AES_ENCRYPTION_ENABLED, AES_KEY, AES_IV};
use crate::network::protocol::FlutterCommand;
use crate::sensors::hal::Esp32Telemetry;

type Aes256CbcEnc = cbc::Encryptor<Aes256>;
type Aes256CbcDec = cbc::Decryptor<Aes256>;

#[derive(Debug, Clone)]
pub enum SimulationCommand {
    ManualDrive { left: i16, right: i16 },
    Conveyor { speed: i16 },
    SetAnchor { active: bool },
    SetHeading { deg: f32 },
    AutoReturn,
    Headlights { active: bool },
    SetWaterCurrent { speed_mps: f32, dir_deg: f32, turbulence: f32 },
    SetWeather { wind_speed_mps: f32, wind_dir_deg: f32, gust_prob: f32 },
    InjectFault { fault_type: String, _duration_sec: f32 },
    TeleportBoat { x: f32, y: f32, yaw_deg: f32, battery_pct: f32 },
    SetHomeCoordinates { x: f32, y: f32 },
    UploadFirmware { source_path: String },
}

pub struct WsServerState {
    pub latest_telemetry: Arc<RwLock<Option<Esp32Telemetry>>>,
    pub cmd_sender: crossbeam_channel::Sender<SimulationCommand>,
    pub drop_packets: Arc<RwLock<bool>>,
    pub connection_severed: Arc<RwLock<bool>>,
}

pub async fn start_websocket_server(
    state: Arc<WsServerState>,
    mut shutdown_rx: tokio::sync::broadcast::Receiver<()>,
) {
    let addr = format!("127.0.0.1:{}", WS_PORT);
    let listener = match TcpListener::bind(&addr).await {
        Ok(l) => {
            println!("[RivSim WebSocket] SITL server listening on ws://{}", addr);
            l
        }
        Err(e) => {
            eprintln!("[RivSim WebSocket] Failed to bind to {}: {}", addr, e);
            return;
        }
    };

    let (tx_broadcast, _) = broadcast::channel::<String>(32);

    // Periodic 1-second telemetry broadcast task
    let state_clone = state.clone();
    let tx_broadcast_clone = tx_broadcast.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(1000));
        loop {
            interval.tick().await;

            // Check if connection is severed by geofence
            let severed = *state_clone.connection_severed.read().await;
            if severed {
                continue;
            }

            // Check if packet dropped by RF attenuation
            let dropped = *state_clone.drop_packets.read().await;
            if dropped {
                continue;
            }

            let opt_telemetry = state_clone.latest_telemetry.read().await.clone();
            if let Some(telemetry) = opt_telemetry {
                if let Ok(mut json_str) = serde_json::to_string(&telemetry) {
                    if AES_ENCRYPTION_ENABLED {
                        json_str = encrypt_payload(&json_str);
                    }
                    let _ = tx_broadcast_clone.send(json_str);
                }
            }
        }
    });

    loop {
        tokio::select! {
            accept_result = listener.accept() => {
                let (stream, peer_addr) = match accept_result {
                    Ok(res) => res,
                    Err(_) => continue,
                };

                let state_inner = state.clone();
                let mut rx_broadcast = tx_broadcast.subscribe();

                tokio::spawn(async move {
                    let ws_stream = match accept_async(stream).await {
                        Ok(ws) => ws,
                        Err(e) => {
                            eprintln!("[RivSim WebSocket] Handshake failed for {}: {}", peer_addr, e);
                            return;
                        }
                    };

                    let (mut ws_sender, mut ws_receiver) = ws_stream.split();

                    loop {
                        tokio::select! {
                            // Outbound Telemetry Message
                            msg_res = rx_broadcast.recv() => {
                                if let Ok(payload) = msg_res {
                                    let msg = tokio_tungstenite::tungstenite::Message::Text(payload);
                                    if ws_sender.send(msg).await.is_err() {
                                        break; // Client disconnected
                                    }
                                }
                            }
                            // Inbound Command from Flutter
                            opt_msg = ws_receiver.next() => {
                                match opt_msg {
                                    Some(Ok(tokio_tungstenite::tungstenite::Message::Text(mut text))) => {
                                        // Ignore incoming command if signal severed by geofence
                                        let severed = *state_inner.connection_severed.read().await;
                                        if severed {
                                            continue;
                                        }

                                        if AES_ENCRYPTION_ENABLED {
                                            if let Some(decrypted) = decrypt_payload(&text) {
                                                text = decrypted;
                                            } else {
                                                continue;
                                            }
                                        }

                                        if let Ok(cmd) = serde_json::from_str::<FlutterCommand>(&text) {
                                            match cmd {
                                                FlutterCommand::ManualDrive { left, right } => {
                                                    let _ = state_inner.cmd_sender.send(SimulationCommand::ManualDrive { left, right });
                                                }
                                                FlutterCommand::Conveyor { speed } => {
                                                    let _ = state_inner.cmd_sender.send(SimulationCommand::Conveyor { speed });
                                                }
                                                FlutterCommand::SetAnchor { active } => {
                                                    let _ = state_inner.cmd_sender.send(SimulationCommand::SetAnchor { active });
                                                }
                                                FlutterCommand::SetHeading { deg } => {
                                                    let _ = state_inner.cmd_sender.send(SimulationCommand::SetHeading { deg });
                                                }
                                                FlutterCommand::AutoReturn => {
                                                    let _ = state_inner.cmd_sender.send(SimulationCommand::AutoReturn);
                                                }
                                                FlutterCommand::Headlights { active } => {
                                                    let _ = state_inner.cmd_sender.send(SimulationCommand::Headlights { active });
                                                }
                                            }
                                        }
                                    }
                                    Some(Ok(tokio_tungstenite::tungstenite::Message::Close(_))) | None => {
                                        break;
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                });
            }
            _ = shutdown_rx.recv() => {
                break;
            }
        }
    }
}

/// AES-256-CBC Encrypt payload with PKCS7 padding, returning hex string
pub fn encrypt_payload(data: &str) -> String {
    let cipher = Aes256CbcEnc::new(&AES_KEY.into(), &AES_IV.into());
    let mut buf = [0u8; 4096];
    let bytes = data.as_bytes();
    if bytes.len() > 4000 {
        return data.to_string();
    }
    buf[..bytes.len()].copy_from_slice(bytes);
    if let Ok(ciphertext) = cipher.encrypt_padded_mut::<Pkcs7>(&mut buf, bytes.len()) {
        hex_encode(ciphertext)
    } else {
        data.to_string()
    }
}

/// AES-256-CBC Decrypt hex payload with PKCS7 unpadding
pub fn decrypt_payload(hex_str: &str) -> Option<String> {
    let mut bytes = hex_decode(hex_str)?;
    let cipher = Aes256CbcDec::new(&AES_KEY.into(), &AES_IV.into());
    let plaintext = cipher.decrypt_padded_mut::<Pkcs7>(&mut bytes).ok()?;
    String::from_utf8(plaintext.to_vec()).ok()
}

fn hex_encode(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02x}", b)).collect()
}

fn hex_decode(hex: &str) -> Option<Vec<u8>> {
    if hex.len() % 2 != 0 {
        return None;
    }
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).ok())
        .collect()
}
