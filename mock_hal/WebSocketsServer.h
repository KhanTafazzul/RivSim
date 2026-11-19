#ifndef WEBSOCKETS_SERVER_H
#define WEBSOCKETS_SERVER_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    WStype_DISCONNECTED,
    WStype_CONNECTED,
    WStype_TEXT,
    WStype_BIN,
    WStype_ERROR
} WStype_t;

typedef void (*WebSocketServerEvent)(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

class WebSocketsServer {
private:
    uint16_t port;
    WebSocketServerEvent cb;
public:
    WebSocketsServer(uint16_t p) : port(p), cb(nullptr) {}

    void begin() {}
    void loop() {}
    void onEvent(WebSocketServerEvent callback) {
        cb = callback;
    }

    bool sendTXT(uint8_t num, const char * text) { return true; }
    bool sendTXT(uint8_t num, uint8_t * payload, size_t length) { return true; }
    void broadcastTXT(const char * text) {}
    void broadcastTXT(uint8_t * payload, size_t length) {}
};

#endif // WEBSOCKETS_SERVER_H

