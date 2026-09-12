#include "websocket_server.h"

// Static instance de dung trong lambda callback cua thu vien WebSockets
static HexapodWebSocket* _wsInstance = nullptr;

void HexapodWebSocket::begin(GaitEngine* gait, ServoController* servo) {
    _gait  = gait;
    _servo = servo;
    _wsInstance = this;

    _ws.begin();
    _ws.onEvent([](uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
        if (!_wsInstance) return;
        switch (type) {
            case WStype_CONNECTED:
                _wsInstance->handleConnect(clientNum);
                break;
            case WStype_DISCONNECTED:
                _wsInstance->handleDisconnect(clientNum);
                break;
            case WStype_TEXT: {
                String msg;
                msg.reserve(length);
                for (size_t i = 0; i < length; i++) msg += (char)payload[i];
                _wsInstance->handleMessage(clientNum, msg);
                break;
            }
            default: break;
        }
    });

    Serial.printf("[WebSocket] Server dang lang nghe tren port %d\n", WS_PORT);
}

void HexapodWebSocket::loop() {
    _ws.loop();
}

void HexapodWebSocket::handleConnect(uint8_t clientNum) {
    _connectedCount++;
    Serial.printf("[WebSocket] Client #%d ket noi. Tong: %d\n", clientNum, _connectedCount);

    JsonDocument doc;
    doc["type"]    = "connected";
    doc["message"] = "Hexapod Ready";
    doc["version"] = "1.0.0";
    sendJson(clientNum, doc);
}

void HexapodWebSocket::handleDisconnect(uint8_t clientNum) {
    if (_connectedCount > 0) _connectedCount--;
    Serial.printf("[WebSocket] Client #%d mat ket noi. Tong: %d\n", clientNum, _connectedCount);

    // An toan: khong con client nao -> dung robot
    if (_connectedCount == 0 && _gait) {
        _gait->stop();
        Serial.println("[WebSocket] Khong con client — da dung robot (safety)");
    }
}

void HexapodWebSocket::handleMessage(uint8_t clientNum, String& payload) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (err) {
        JsonDocument errDoc;
        errDoc["type"]    = "error";
        errDoc["message"] = "Invalid JSON";
        sendJson(clientNum, errDoc);
        return;
    }

    const char* type = doc["type"];
    if (!type) return;

    if (strcmp(type, "move") == 0) {
        float vx    = doc["vx"]    | 0.0f;
        float vy    = doc["vy"]    | 0.0f;
        float omega = doc["omega"] | 0.0f;
        if (_gait) _gait->setVelocity(vx, vy, omega);

    } else if (strcmp(type, "stop") == 0) {
        if (_gait) _gait->stop();

    } else if (strcmp(type, "mode") == 0) {
        const char* modeName = doc["value"] | "STANDARD";
        int modeIdx = findModeByName(modeName);
        if (modeIdx >= 0 && _gait) {
            _gait->setMode((uint8_t)modeIdx);
            JsonDocument resp;
            resp["type"] = "ack";
            resp["mode"] = modeName;
            sendJson(clientNum, resp);
        } else {
            JsonDocument errDoc;
            errDoc["type"]    = "error";
            errDoc["message"] = "Unknown mode";
            sendJson(clientNum, errDoc);
        }

    } else if (strcmp(type, "gait") == 0) {
        const char* gaitName = doc["value"] | "TRIPOD";
        if (_gait) {
            GaitType gt = (strcmp(gaitName, "WAVE") == 0) ? GaitType::WAVE : GaitType::TRIPOD;
            _gait->setGaitType(gt);
            JsonDocument resp;
            resp["type"] = "ack";
            resp["gait"] = gaitName;
            sendJson(clientNum, resp);
        }

    } else if (strcmp(type, "pose") == 0) {
        float height = doc["height"] | 120.0f;
        float pitch  = doc["pitch"]  | 0.0f;
        float roll   = doc["roll"]   | 0.0f;
        float yaw    = doc["yaw"]    | 0.0f;
        if (_gait) _gait->setBodyPose(height, pitch, roll, yaw);

    } else if (strcmp(type, "calib") == 0) {
        // Calibration: chinh trim (do) cho tung servo, luu tam thoi trong RAM.
        // { "type": "calib", "servo": 0, "trim": 5 }
        int servoIdx = doc["servo"] | -1;
        int trim     = doc["trim"]  | 0;
        if (servoIdx >= 0 && servoIdx < 18) {
            SERVO_CAL[servoIdx].trim = (int16_t)trim;
            if (_servo) _servo->setAngle((uint8_t)servoIdx, _servo->getCurrentAngle((uint8_t)servoIdx));
            JsonDocument resp;
            resp["type"]  = "ack";
            resp["servo"] = servoIdx;
            resp["trim"]  = trim;
            sendJson(clientNum, resp);
        }

    } else if (strcmp(type, "ping") == 0) {
        JsonDocument resp;
        resp["type"]   = "pong";
        resp["uptime"] = millis();
        sendJson(clientNum, resp);

    } else if (strcmp(type, "speed") == 0) {
        float speed = doc["value"] | 1.0f;
        if (_gait) _gait->setSpeedFactor(speed);

    } else {
        JsonDocument errDoc;
        errDoc["type"]    = "error";
        errDoc["message"] = "Unknown command";
        sendJson(clientNum, errDoc);
    }
}

void HexapodWebSocket::sendJson(uint8_t clientNum, JsonDocument& doc) {
    String output;
    serializeJson(doc, output);
    _ws.sendTXT(clientNum, output);
}

void HexapodWebSocket::broadcastStatus(float batteryVoltage, bool touchSensors[6], uint8_t modeIndex) {
    if (_connectedCount == 0) return;

    JsonDocument doc;
    doc["type"]    = "status";
    doc["battery"] = batteryVoltage;
    doc["mode"]    = MODE_NAMES[modeIndex];

    JsonArray touch = doc["touch"].to<JsonArray>();
    for (int i = 0; i < 6; i++) {
        touch.add(touchSensors[i] ? 1 : 0);
    }

    String output;
    serializeJson(doc, output);
    _ws.broadcastTXT(output);
}

int HexapodWebSocket::findModeByName(const char* name) {
    for (int i = 0; i < MODE_COUNT; i++) {
        if (strcmp(MODE_NAMES[i], name) == 0) return i;
    }
    return -1;
}
