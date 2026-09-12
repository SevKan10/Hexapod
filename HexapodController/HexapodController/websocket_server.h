#pragma once
#include <Arduino.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "gait_engine.h"
#include "servo_controller.h"

// ============================================================
// HexapodWebSocket — Nhan lenh JSON tu mobile app qua WebSocket
//
// Lenh ho tro:
//   { "type": "move",  "vx": 0.5, "vy": 0.0, "omega": 0.1 }
//   { "type": "stop" }
//   { "type": "mode",  "value": "STANDARD" }
//   { "type": "gait",  "value": "TRIPOD" }   // hoac "WAVE"
//   { "type": "pose",  "height": 120, "pitch": 0, "roll": 0, "yaw": 0 }
//   { "type": "calib", "servo": 0, "trim": 5 }
//   { "type": "speed", "value": 1.0 }
//   { "type": "ping" }
//
// Phan hoi (JSON):
//   { "type": "connected", "message": "Hexapod Ready", "version": "1.0.0" }
//   { "type": "pong", "uptime": 12345 }
//   { "type": "status", "battery": 7.4, "touch": [0,0,0,0,0,0], "mode": "STANDARD" }
//   { "type": "ack", ... }
//   { "type": "error", "message": "..." }
// ============================================================
class HexapodWebSocket {
public:
    void begin(GaitEngine* gait, ServoController* servo);
    void loop();

    void broadcastStatus(float batteryVoltage, bool touchSensors[6], uint8_t modeIndex);

    uint8_t connectedClients() const { return _connectedCount; }

private:
    WebSocketsServer _ws = WebSocketsServer(WS_PORT);
    GaitEngine*      _gait  = nullptr;
    ServoController* _servo = nullptr;
    uint8_t          _connectedCount = 0;

    void handleMessage(uint8_t clientNum, String& payload);
    void handleConnect(uint8_t clientNum);
    void handleDisconnect(uint8_t clientNum);

    void sendJson(uint8_t clientNum, JsonDocument& doc);

    int findModeByName(const char* name);
};
