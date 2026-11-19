#ifndef HAL_BRIDGE_H
#define HAL_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared C-compatible structure between Rust simulation engine and ESP32 C++ firmware
#pragma pack(push, 1)
typedef struct {
    uint64_t sim_micros;         // Simulation clock in microseconds

    // Analog Inputs (12-bit ADC: 0 - 4095)
    uint16_t adc_turbidity_pin34; // River turbidity sensor on GPIO 34
    uint16_t adc_battery_pin35;   // Battery divider on GPIO 35
    uint16_t adc_ldr_pin36;       // Ambient light LDR on GPIO 36
    uint16_t adc_ntc_pin39;       // BTS7960 NTC thermistor on GPIO 39

    // I2C 0x68 (MPU-6050)
    int16_t mpu6050_gyro_z_raw;   // Gyro Z axis angular rate (reg 0x47-0x48)
    int16_t mpu6050_accel_x_raw;  // Accel X axis (reg 0x3B-0x3C)
    int16_t mpu6050_accel_y_raw;  // Accel Y axis (reg 0x3D-0x3E)

    // UART1 GPS NMEA Buffer (GPIO 16/17)
    char     uart1_nmea_buf[128];
    uint8_t  uart1_nmea_len;
    uint8_t  uart1_read_pos;

    // Actuator & Output Pins (Controlled by Firmware)
    int16_t  pwm_motor_left;      // Left motor PWM (-255 to +255, GPIO 32/33)
    int16_t  pwm_motor_right;     // Right motor PWM (-255 to +255, GPIO 25/26)
    int16_t  pwm_conveyor;        // Front conveyor speed (0 to 255, GPIO 27/14)
    uint8_t  gpio_headlights;     // Virtual headlights toggle (0 or 1, GPIO 4)

    // Firmware Status & Failsafe Flags
    uint8_t  failsafe_active;     // 1 if failsafe is engaged
    uint8_t  firmware_state;      // 0=MANUAL, 1=HEADING_HOLD, 2=ANCHOR, 3=LOITER, 4=RTH
    uint8_t  thermal_throttled;   // 1 if driver thermal clamp is active
    float    target_heading_deg;  // Current target heading in degrees
} HalBridgeState;
#pragma pack(pop)

// Global bridge pointer accessible to mock C++ libraries
extern HalBridgeState* g_hal_state;

#ifdef __cplusplus
}
#endif

#endif // HAL_BRIDGE_H

