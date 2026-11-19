// ESP32 River-Cleaning Catamaran Autonomous Firmware
// Compiled natively in RivSim or flashed directly onto physical ESP32
#include "Arduino.h"
#include "Wire.h"
#include "HardwareSerial.h"
#include "WiFi.h"
#include "WebSocketsServer.h"
#include "TinyGPS++.h"
#include "hal_bridge.h"

// Hardware Pin Definitions
#define PIN_MOTOR_L_PWM   32
#define PIN_MOTOR_L_DIR   33
#define PIN_MOTOR_R_PWM   25
#define PIN_MOTOR_R_DIR   26
#define PIN_CONVEYOR_PWM  27
#define PIN_HEADLIGHTS    4

#define PIN_ADC_TURBIDITY 34
#define PIN_ADC_BATTERY   35
#define PIN_ADC_LDR       36
#define PIN_ADC_NTC       39

// Global Singletons
HalBridgeState* g_hal_state = nullptr;
MockSerial Serial;
HardwareSerial Serial1(1);
MockWiFi WiFi;
TwoWire Wire;
TinyGPSPlus gps;

// Firmware State Machine
enum BoatMode {
    MODE_MANUAL = 0,
    MODE_HEADING_HOLD = 1,
    MODE_PARKING_ANCHOR = 2,
    MODE_LOITER = 3,
    MODE_RETURN_TO_HOME = 4
};

static BoatMode current_mode = MODE_HEADING_HOLD;
static float target_heading = 0.0f;
static float anchor_lat = 0.0f;
static float anchor_lon = 0.0f;
static float home_lat = 12.971598f;
static float home_lon = 77.594562f;

static unsigned long last_signal_time = 0;
static unsigned long loiter_start_time = 0;
static bool failsafe_triggered = false;

// PID Heading-Hold Controller
static float Kp = 2.4f;
static float Ki = 0.05f;
static float Kd = 0.8f;
static float integral_error = 0.0f;
static float prev_error = 0.0f;
static unsigned long last_pid_time = 0;

// Motor Trim Multipliers (Hardware Calibration)
static float trim_left = 1.0f;
static float trim_right = 0.96f;

// Angle difference in range [-180, +180]
static float angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// Haversine distance in meters
static float calculate_distance(float lat1, float lon1, float lat2, float lon2) {
    float dlat = (lat2 - lat1) * (PI / 180.0f);
    float dlon = (lon2 - lon1) * (PI / 180.0f);
    float a = sin(dlat / 2.0f) * sin(dlat / 2.0f) +
              cos(lat1 * (PI / 180.0f)) * cos(lat2 * (PI / 180.0f)) *
              sin(dlon / 2.0f) * sin(dlon / 2.0f);
    float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
    return 6371000.0f * c;
}

// Bearing from point 1 to point 2 in degrees [0, 360)
static float calculate_bearing(float lat1, float lon1, float lat2, float lon2) {
    float dlon = (lon2 - lon1) * (PI / 180.0f);
    float y = sin(dlon) * cos(lat2 * (PI / 180.0f));
    float x = cos(lat1 * (PI / 180.0f)) * sin(lat2 * (PI / 180.0f)) -
              sin(lat1 * (PI / 180.0f)) * cos(lat2 * (PI / 180.0f)) * cos(dlon);
    float brng = atan2(y, x) * (180.0f / PI);
    if (brng < 0.0f) brng += 360.0f;
    return brng;
}

extern "C" {

void firmware_init(HalBridgeState* bridge) {
    g_hal_state = bridge;
    last_signal_time = millis();
    last_pid_time = millis();
}

void firmware_setup() {
    pinMode(PIN_HEADLIGHTS, OUTPUT);
    Wire.begin(21, 22);
    Serial1.begin(9600, 0, 16, 17);
}

void firmware_loop() {
    unsigned long now = millis();

    // 1. Ingest GPS NMEA Stream from UART1
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }

    // 2. Read MPU-6050 Gyro Z over I2C
    Wire.beginTransmission(0x68);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2);
    int16_t gyro_z = 0;
    if (Wire.available() >= 2) {
        gyro_z = (Wire.read() << 8) | Wire.read();
    }

    // 3. Sensor Checks: LDR Auto-Headlights
    int ldr_raw = analogRead(PIN_ADC_LDR);
    // ADC 12-bit: lower value = darker
    if (ldr_raw < 1200) {
        digitalWrite(PIN_HEADLIGHTS, HIGH);
    } else {
        digitalWrite(PIN_HEADLIGHTS, LOW);
    }

    // 4. BTS7960 Driver Thermal Protection
    int ntc_raw = analogRead(PIN_ADC_NTC);
    int max_pwm_allowed = 255;
    if (ntc_raw > 2800) { // Approx > 70°C
        max_pwm_allowed = 150;
        if (g_hal_state) g_hal_state->thermal_throttled = 1;
    } else {
        if (g_hal_state) g_hal_state->thermal_throttled = 0;
    }

    // 5. Failsafe State Machine
    // Check if Wi-Fi / WebSocket signal dropped
    if (g_hal_state && g_hal_state->failsafe_active) {
        if (current_mode != MODE_LOITER && current_mode != MODE_RETURN_TO_HOME) {
            current_mode = MODE_LOITER;
            loiter_start_time = now;
        }
    }

    if (current_mode == MODE_LOITER) {
        // Loiter for 30 seconds into current, then enter RTH
        if (now - loiter_start_time > 30000) {
            current_mode = MODE_RETURN_TO_HOME;
        }
    }

    // 6. Navigation Control & PID Computation
    float current_yaw = 0.0f;
    if (gps.course.isValid()) {
        current_yaw = (float)gps.course.deg();
    }

    int base_throttle = 0;
    float desired_heading = target_heading;

    if (current_mode == MODE_RETURN_TO_HOME) {
        if (gps.location.isValid()) {
            float dist = calculate_distance(gps.location.lat(), gps.location.lng(), home_lat, home_lon);
            if (dist > 1.5f) {
                desired_heading = calculate_bearing(gps.location.lat(), gps.location.lng(), home_lat, home_lon);
                base_throttle = 180;
            } else {
                base_throttle = 0; // Reached home!
            }
        }
    } else if (current_mode == MODE_PARKING_ANCHOR) {
        if (gps.location.isValid()) {
            if (anchor_lat == 0.0f && anchor_lon == 0.0f) {
                anchor_lat = gps.location.lat();
                anchor_lon = gps.location.lng();
            }
            float drift = calculate_distance(gps.location.lat(), gps.location.lng(), anchor_lat, anchor_lon);
            if (drift > 3.0f) {
                desired_heading = calculate_bearing(gps.location.lat(), gps.location.lng(), anchor_lat, anchor_lon);
                base_throttle = 160;
            } else {
                base_throttle = 0;
            }
        }
    } else if (current_mode == MODE_LOITER) {
        base_throttle = 80; // Low forward throttle holding position
    }

    // PID Heading Correction
    float dt = (float)(now - last_pid_time) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
    last_pid_time = now;

    float err = angle_diff(desired_heading, current_yaw);
    integral_error += err * dt;
    // Anti-windup clamping
    if (integral_error > 50.0f) integral_error = 50.0f;
    if (integral_error < -50.0f) integral_error = -50.0f;

    float derivative = (err - prev_error) / dt;
    prev_error = err;

    float steering = (Kp * err) + (Ki * integral_error) + (Kd * derivative);
    if (steering > 120.0f) steering = 120.0f;
    if (steering < -120.0f) steering = -120.0f;

    int left_cmd = (int)((base_throttle - steering) * trim_left);
    int right_cmd = (int)((base_throttle + steering) * trim_right);

    // Clamp by thermal protection
    left_cmd = constrain(left_cmd, -max_pwm_allowed, max_pwm_allowed);
    right_cmd = constrain(right_cmd, -max_pwm_allowed, max_pwm_allowed);

    // Apply to virtual actuators
    analogWrite(PIN_MOTOR_L_PWM, left_cmd);
    analogWrite(PIN_MOTOR_R_PWM, right_cmd);

    // Update Bridge Diagnostics
    if (g_hal_state) {
        g_hal_state->firmware_state = (uint8_t)current_mode;
        g_hal_state->target_heading_deg = desired_heading;
    }
}

} // extern "C"

