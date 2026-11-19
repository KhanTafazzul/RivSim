#ifndef ARDUINO_H
#define ARDUINO_H

#include "hal_bridge.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define PI 3.14159265358979323846

#ifdef __cplusplus
#include <string>
#include <iostream>

inline unsigned long millis() {
    if (!g_hal_state) return 0;
    return (unsigned long)(g_hal_state->sim_micros / 1000ULL);
}

inline unsigned long micros() {
    if (!g_hal_state) return 0;
    return (unsigned long)(g_hal_state->sim_micros);
}

inline void delay(unsigned long ms) {
    // In headless or step simulation, spin or no-op
}

inline void delayMicroseconds(unsigned int us) {
}

inline void pinMode(uint8_t pin, uint8_t mode) {
    // Virtual pin setup
}

inline void digitalWrite(uint8_t pin, uint8_t val) {
    if (!g_hal_state) return;
    if (pin == 4) {
        g_hal_state->gpio_headlights = (val == HIGH) ? 1 : 0;
    }
}

inline int digitalRead(uint8_t pin) {
    return LOW;
}

inline int analogRead(uint8_t pin) {
    if (!g_hal_state) return 0;
    switch (pin) {
        case 34: return g_hal_state->adc_turbidity_pin34;
        case 35: return g_hal_state->adc_battery_pin35;
        case 36: return g_hal_state->adc_ldr_pin36;
        case 39: return g_hal_state->adc_ntc_pin39;
        default: return 0;
    }
}

inline void analogWrite(uint8_t pin, int val) {
    if (!g_hal_state) return;
    if (pin == 32 || pin == 33) {
        g_hal_state->pwm_motor_left = (int16_t)val;
    } else if (pin == 25 || pin == 26) {
        g_hal_state->pwm_motor_right = (int16_t)val;
    } else if (pin == 27 || pin == 14) {
        g_hal_state->pwm_conveyor = (int16_t)val;
    }
}

inline void ledcWrite(uint8_t channel, uint32_t duty) {
    if (!g_hal_state) return;
    if (channel == 0) g_hal_state->pwm_motor_left = (int16_t)duty;
    else if (channel == 1) g_hal_state->pwm_motor_right = (int16_t)duty;
    else if (channel == 2) g_hal_state->pwm_conveyor = (int16_t)duty;
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if (x < a) return a;
    if (b < x) return b;
    return x;
}

class MockSerial {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) {}
    void print(int n) {}
    void print(float f) {}
    void println(const char* s = "") {}
    void println(int n) {}
    void println(float f) {}
};

extern MockSerial Serial;

#endif // __cplusplus
#endif // ARDUINO_H

