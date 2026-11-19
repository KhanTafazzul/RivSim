#ifndef WIRE_H
#define WIRE_H

#include "hal_bridge.h"
#include <stdint.h>

class TwoWire {
private:
    uint8_t target_addr;
    uint8_t reg_addr;
    uint8_t read_counter;
    uint8_t buffer[14];
    uint8_t buffer_len;

public:
    TwoWire() : target_addr(0), reg_addr(0), read_counter(0), buffer_len(0) {}

    void begin() {}
    void begin(int sda, int scl) {}

    void beginTransmission(uint8_t addr) {
        target_addr = addr;
        reg_addr = 0;
    }

    void write(uint8_t b) {
        reg_addr = b;
    }

    uint8_t endTransmission(bool sendStop = true) {
        return 0;
    }

    uint8_t requestFrom(uint8_t addr, uint8_t quantity) {
        read_counter = 0;
        buffer_len = quantity;
        if (addr == 0x68 && g_hal_state) {
            // Fill MPU-6050 registers based on reg_addr
            // 0x3B: Accel X(2), Accel Y(2), Accel Z(2), Temp(2), Gyro X(2), Gyro Y(2), Gyro Z(2)
            int16_t ax = g_hal_state->mpu6050_accel_x_raw;
            int16_t ay = g_hal_state->mpu6050_accel_y_raw;
            int16_t gz = g_hal_state->mpu6050_gyro_z_raw;

            if (reg_addr == 0x3B || reg_addr == 0x47) {
                buffer[0] = (uint8_t)(ax >> 8);
                buffer[1] = (uint8_t)(ax & 0xFF);
                buffer[2] = (uint8_t)(ay >> 8);
                buffer[3] = (uint8_t)(ay & 0xFF);
                buffer[4] = 0x40; // 1g Z
                buffer[5] = 0x00;
                buffer[6] = 0x00; // Temp
                buffer[7] = 0x00;
                buffer[8] = 0x00; // Gyro X
                buffer[9] = 0x00;
                buffer[10] = 0x00; // Gyro Y
                buffer[11] = 0x00;
                buffer[12] = (uint8_t)(gz >> 8);
                buffer[13] = (uint8_t)(gz & 0xFF);
                
                if (reg_addr == 0x47) {
                    buffer[0] = (uint8_t)(gz >> 8);
                    buffer[1] = (uint8_t)(gz & 0xFF);
                }
            }
        }
        return quantity;
    }

    int read() {
        if (read_counter < buffer_len && read_counter < 14) {
            return buffer[read_counter++];
        }
        return -1;
    }

    int available() {
        return (buffer_len > read_counter) ? (buffer_len - read_counter) : 0;
    }
};

extern TwoWire Wire;

#endif // WIRE_H

