#ifndef WIFI_H
#define WIFI_H

#include <stdint.h>

enum wl_status_t {
    WL_NO_SHIELD = 255,
    WL_IDLE_STATUS = 0,
    WL_NO_SSID_AVAIL = 1,
    WL_SCAN_COMPLETED = 2,
    WL_CONNECTED = 3,
    WL_CONNECT_FAILED = 4,
    WL_CONNECTION_LOST = 5,
    WL_DISCONNECTED = 6
};

class MockWiFi {
public:
    void mode(int m) {}
    void begin(const char* ssid, const char* passphrase = nullptr) {}
    wl_status_t status() { return WL_CONNECTED; }
    int32_t RSSI() { return -65; }
    const char* localIP() { return "192.168.4.1"; }
    bool softAP(const char* ssid, const char* passphrase = nullptr) { return true; }
};

extern MockWiFi WiFi;

#endif // WIFI_H

