//! Radio Attenuation, Wi-Fi RSSI Degradation, and Geofence Failsafe
//! Implements Log-Distance Path Loss model and packet drop probabilities.

use rand::Rng;
use crate::config::{BASE_STATION_X, BASE_STATION_Y, WIFI_DROP_RADIUS_M, GEOFENCE_RADIUS_M};

#[derive(Debug, Clone)]
pub struct RadioSubsystem {
    pub distance_to_base_m: f32,
    pub rssi_dbm: i32,
    pub packet_drop_probability: f32,
    pub connection_severed: bool,
    pub geofence_breached: bool,
    pub fault_wifi_drop: bool,
}

impl RadioSubsystem {
    pub fn new() -> Self {
        Self {
            distance_to_base_m: 0.0,
            rssi_dbm: -38,
            packet_drop_probability: 0.0,
            connection_severed: false,
            geofence_breached: false,
            fault_wifi_drop: false,
        }
    }

    /// Update radio link status based on boat world coordinates (x, y)
    pub fn update(&mut self, boat_x: f32, boat_y: f32) {
        let dx = boat_x - BASE_STATION_X;
        let dy = boat_y - BASE_STATION_Y;
        let dist = (dx * dx + dy * dy).sqrt().max(0.5);
        self.distance_to_base_m = dist;

        let mut rng = rand::thread_rng();

        // 1. Log-Distance Path Loss: RSSI(d) = RSSI_0 - 10 * n * log10(d / d0) + X_sigma
        // RSSI_0 = -36 dBm at 1m, path loss exponent n = 2.4 (open water environment)
        let path_loss = 10.0 * 2.4 * (dist / 1.0).log10();
        let shadow_fading = rng.gen_range(-2.0..2.0);
        let raw_rssi = -36.0 - path_loss + shadow_fading;
        self.rssi_dbm = raw_rssi.clamp(-98.0, -25.0) as i32;

        // 2. Geofence & Disconnect Boundary
        self.geofence_breached = dist > GEOFENCE_RADIUS_M;

        // Injected fault override
        if self.fault_wifi_drop {
            self.packet_drop_probability = 1.0;
            self.connection_severed = true;
            return;
        }

        // 3. Packet Drop Probability (Sigmoid curve past 35m)
        if dist < WIFI_DROP_RADIUS_M {
            self.packet_drop_probability = 0.01; // Negligible packet loss within nominal zone
            self.connection_severed = false;
        } else if dist < GEOFENCE_RADIUS_M {
            // Drop rate escalates from 5% to 90% as boat approaches 50m
            let t = (dist - WIFI_DROP_RADIUS_M) / (GEOFENCE_RADIUS_M - WIFI_DROP_RADIUS_M);
            self.packet_drop_probability = (0.05 + 0.85 * t * t).clamp(0.0, 1.0);
            self.connection_severed = false;
        } else {
            // Beyond 50m geofence: complete 100% loss and link severance
            self.packet_drop_probability = 1.0;
            self.connection_severed = true;
        }
    }

    /// Check if a WebSocket packet should be dropped according to current RF propagation
    pub fn should_drop_packet(&self) -> bool {
        if self.connection_severed || self.fault_wifi_drop {
            return true;
        }
        if self.packet_drop_probability <= 0.0 {
            return false;
        }
        rand::thread_rng().gen_bool(self.packet_drop_probability as f64)
    }
}

