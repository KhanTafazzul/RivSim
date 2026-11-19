# RivSim: Autonomous Catamaran Digital Twin & SITL Physics Engine

A high-fidelity 2D physics simulation, Software-In-The-Loop (SITL) platform, ESP32 C++ firmware execution host, and PyTorch Reinforcement Learning dataset generator written in pure Rust.

Designed as an exact digital twin for an autonomous river-cleaning catamaran boat powered by an ESP32 microcontroller, interfacing with a Flutter mobile/desktop telemetry dashboard and PyTorch RL navigation models.

---

## Key Features

- **Physics Engine (`rapier2d` & SI Units)**:
  - Catamaran compound rigid body: twin 6" ($0.1524\text{ m}$) PVC pontoons, $1.2\text{ m}$ length, $0.65\text{ m}$ center-to-center spacing.
  - Front conveyor belt: $0.8532\text{ m}$ wide (extending 1 inch beyond each pontoon).
  - Concentrated offset center-of-gravity (ESP32 and battery placement).
  - Differential skid steering: twin rear paddle wheels producing linear thrust and yaw moment.
  - Hydrodynamic drag (longitudinal streamlining vs. high lateral keel resistance) and aerodynamic wind sail forces.
- **Dynamic Environmental Physics**:
  - River vector field with parabolic flow profile (fastest at centerline, slower friction near banks).
  - Procedural turbulent vortex eddies drifting downstream.
  - Stochastic wind gusts using an Ornstein-Uhlenbeck drift model.
  - Radio attenuation (Log-Distance Path Loss model): nominal zone ($<35\text{ m}$), packet drop zone ($35\text{m} - 50\text{m}$), and complete connection severing past the $50\text{ m}$ geofence.
- **ESP32 Sensor Hardware Emulation**:
  - Virtual NEO-6M GPS with WGS84 coordinate projection, $1.5\text{m} - 3.0\text{m}$ satellite bounce noise at low speeds, and `$GPRMC` NMEA output.
  - Virtual MPU-6050 IMU with absolute yaw, angular rate, and I2C `0x68` registers.
  - Virtual 3S Battery model with load-dependent voltage sag and ADC pin 35 readings.
  - Spatial river turbidity field (NTU) for Flutter water quality heatmaps.
  - LDR ambient light sensor with automatic virtual headlights trigger below 80 Lux.
  - BTS7960 motor driver NTC thermal model with 70°C throttle clamping protection.
- **ESP32 C++ Firmware Runner**:
  - Drop-in mock headers (`mock_hal/`: `Arduino.h`, `Wire.h`, `HardwareSerial.h`, `WiFi.h`, etc.).
  - `millis()` and `micros()` locked to Rapier2D simulation time for identical execution at 1x visual and 60,000x headless speeds.
  - Dynamic sketch compilation via `cl.exe`/`g++`/`clang++` with hot-loading via `libloading`.
  - Native Rust fallback controller: PID heading-hold with anti-windup, motor trim, 3m parking anchor drift recovery, 30s loiter, and GPS Auto-Return-to-Home (RTH).
- **Tokio Networking Stack**:
  - Local WebSocket server on `ws://127.0.0.1:81` for the Flutter dashboard with optional `AES-256-CBC` encryption.
  - Embedded Model Context Protocol (MCP) server on `http://127.0.0.1:8000/mcp` (`rmcp` + `axum` + `schemars`) exposing 9 AI test orchestration tools.
- **PyTorch RL Return-to-Home Dataset Generator**:
  - Rayon 4-core parallel headless execution writing 28-column CSV trajectories at $>10,000$ steps/sec.
- **Developer Visualizer (`macroquad`)**:
  - Top-down dark-space view, animated river flow vectors, foam particle trails, wind streaks, Wi-Fi coverage rings, headlight beams, and live telemetry HUD.

---

## Architecture Overview

```
RivSim/
├── Cargo.toml
├── mock_hal/                   # ESP32 C++ Mock Headers (Arduino, Wire, Serial, WiFi)
├── firmware/                   # Sample ESP32 firmware sketch (boat_firmware.cpp)
├── src/
│   ├── main.rs                 # CLI entry point (clap), mode routing
│   ├── config.rs               # Locked physical dimensions & parameters
│   ├── dataset.rs              # 28-column RL CSV dataset logger & rewards
│   ├── headless.rs             # Rayon 4-core parallel batch runner
│   ├── firmware_host/          # C++ dynamic compilation & libloading host
│   ├── physics/                # Rapier2D catamaran, river vectors, radio path loss
│   ├── sensors/                # GPS, IMU, Battery, Turbidity, LDR, Thermal, Firmware
│   ├── network/                # WebSocket server (:81) & Native MCP server (:8000/mcp)
│   └── render/                 # Macroquad visualizer, boat render, HUD dashboard
└── tests/
    └── sim_tests.rs            # Integration test suite (13 passing tests)
```

---

## How to Build and Run

### Prerequisites
- Modern Rust toolchain (Rust 1.80+)
- Windows, macOS, or Linux

### 1. Interactive Visual Simulation Mode
Launches the 2D visualizer, SITL WebSocket server on port 81, and MCP server on port 8000:
```bash
cargo run -- --mode visual
```

### 2. Parallel Headless RL Dataset Generation
Bypasses graphics and networking, spawning 4 parallel Rapier2D worlds across CPU cores:
```bash
cargo run --release -- --mode headless --episodes 1000 --rows 500000 --out return_dataset.csv
```

### 3. Run Automated Tests
```bash
cargo test
```

---

## Visualizer Controls

| Key | Action |
| :--- | :--- |
| **W / S / Up / Down** | Forward Throttle / Reverse Throttle |
| **A / D / Left / Right** | Differential Yaw Steering |
| **Space** | Emergency All Stop |
| **C** | Toggle Front Conveyor Belt |
| **H** | Toggle Headlights |
| **F** | Toggle Follow-Boat Camera Mode |
| **L** | Hot-Load C++ Firmware Sketch (`firmware/boat_firmware.cpp`) |
| **1 / 2 / 3 / 4** | Switch Firmware Modes: 1=HeadingHold, 2=Anchor, 3=Loiter, 4=RTH |
| **Right Mouse Drag** | Pan Viewport Camera |
| **Mouse Scroll Wheel** | Zoom In / Out |

---

## Model Context Protocol (MCP) Tools

The embedded MCP server (`http://127.0.0.1:8000/mcp`) provides 9 live AI test orchestration tools:
1. `upload_esp32_firmware(source_path, inline_code_optional)`
2. `teleport_boat(x, y, yaw_deg, battery_pct)`
3. `set_home_coordinates(x, y)`
4. `set_water_current(speed_mps, direction_deg, turbulence)`
5. `set_weather(wind_speed_mps, wind_dir_deg, gust_probability)`
6. `inject_fault(fault_type, duration_sec)` (`motor_jam_left`, `motor_jam_right`, `gps_spike`, `wifi_drop`)
7. `get_telemetry_snapshot()`
8. `run_return_episode(...)`
9. `run_headless_return_dataset(num_episodes, target_rows, output_csv_path, enable_exploration)`

---

## Author & License

Developed for autonomous river-cleaning research. MIT License.

