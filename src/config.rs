//! Locked Physical Dimensions, Environmental Parameters, and System Constants

// --- Physical Boat Dimensions (MKS Units) ---
pub const PONTOON_DIAMETER_M: f32 = 0.1524;  // 6 inches PVC pipe
pub const PONTOON_LENGTH_M: f32 = 1.2000;    // 1.2 meters
pub const PONTOON_SPACING_M: f32 = 0.6500;   // 0.65m center-to-center

// Front conveyor extends 1 inch (0.0254m) beyond outer edge of each pontoon
// Outer edge span = 0.65 + 0.1524 = 0.8024m. Width = 0.8024 + (2 * 0.0254) = 0.8532m
pub const CONVEYOR_WIDTH_M: f32 = 0.8532;
pub const CONVEYOR_LENGTH_M: f32 = 0.4500;

// Paddle Wheels
pub const PADDLE_DIAMETER_M: f32 = 0.2800;   // 28cm diameter wheels
pub const PADDLE_SPACING_M: f32 = 0.3500;    // Lateral distance between paddle centers

// Mass & Inertia
pub const BOAT_MASS_KG: f32 = 14.5;          // Pontoons + frame + conveyor + electronics
pub const COG_OFFSET_X: f32 = 0.03;          // Electronics / battery offset X (meters)
pub const COG_OFFSET_Y: f32 = -0.05;         // Electronics / battery offset Y (meters)

// Hydrodynamic & Aerodynamic Drag Coefficients
pub const CD_LONGITUDINAL: f32 = 0.18;       // Streamlined forward pontoon drag
pub const CD_LATERAL: f32 = 1.35;            // High lateral resistance against drift
pub const CD_CONVEYOR_WATER: f32 = 0.45;     // Extra drag plate in water
pub const CD_AERO_WIND: f32 = 0.65;          // Front conveyor & waste box wind catcher
pub const WATER_DENSITY: f32 = 1000.0;       // kg/m^3
pub const AIR_DENSITY: f32 = 1.225;          // kg/m^3

// Thrust Model
pub const MOTOR_THRUST_COEFF: f32 = 0.095;   // Newtons per PWM unit (255 PWM -> ~24.2 N)

// --- Radio & Geofence Coordinates ---
pub const BASE_STATION_X: f32 = 0.0;
pub const BASE_STATION_Y: f32 = 0.0;
pub const WIFI_DROP_RADIUS_M: f32 = 35.0;    // Packet drop starts
pub const GEOFENCE_RADIUS_M: f32 = 50.0;     // Connection severing boundary

// --- GPS Reference Origin (WGS84) ---
pub const ORIGIN_LATITUDE: f64 = 12.971598;
pub const ORIGIN_LONGITUDE: f64 = 77.594562;
pub const METERS_PER_DEGREE_LAT: f64 = 111320.0;

// --- Network & Encryption Configuration ---
pub const WS_PORT: u16 = 81;
pub const MCP_PORT: u16 = 8000;
pub const AES_ENCRYPTION_ENABLED: bool = false; // Toggle for encrypted Flutter telemetry
pub const AES_KEY: [u8; 32] = [
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10,
    0xA0, 0xB1, 0xC2, 0xD3, 0xE4, 0xF5, 0x06, 0x17,
    0x28, 0x39, 0x4A, 0x5B, 0x6C, 0x7D, 0x8E, 0x9F,
];
pub const AES_IV: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
];

