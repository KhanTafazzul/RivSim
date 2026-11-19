#ifndef HARDWARE_SERIAL_H
#define HARDWARE_SERIAL_H

#include "hal_bridge.h"
#include <stdint.h>

class HardwareSerial {
private:
    int uart_nr;
public:
    HardwareSerial(int nr) : uart_nr(nr) {}

    void begin(unsigned long baud, uint32_t config = 0, int rxPin = -1, int txPin = -1) {}

    int available() {
        if (!g_hal_state) return 0;
        if (g_hal_state->uart1_read_pos < g_hal_state->uart1_nmea_len) {
            return g_hal_state->uart1_nmea_len - g_hal_state->uart1_read_pos;
        }
        return 0;
    }

    int read() {
        if (!g_hal_state) return -1;
        if (g_hal_state->uart1_read_pos < g_hal_state->uart1_nmea_len) {
            return (uint8_t)g_hal_state->uart1_nmea_buf[g_hal_state->uart1_read_pos++];
        }
        return -1;
    }

    size_t write(uint8_t c) { return 1; }
    void print(const char* s) {}
    void println(const char* s = "") {}
};

extern HardwareSerial Serial1;

#endif // HARDWARE_SERIAL_H

