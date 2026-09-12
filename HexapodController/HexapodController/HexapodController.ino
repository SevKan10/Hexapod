// ============================================================
// HexapodController — Firmware ESP32 cho hexapod 18 servo
//
// Kien truc:
//   Mobile App (WiFi) <--WebSocket(JSON)--> ESP32 (AP mode)
//                                              |
//                    GaitEngine -> Kinematics (IK) -> ServoController
//                                              |
//                                       2x PCA9685 (I2C)
//
// HUONG DAN NAP CODE (Arduino IDE):
//   1. Copy toan bo thu muc "HexapodController" vao thu muc sketch
//      (vi du: Documents/Arduino/HexapodController/)
//   2. Mo file HexapodController.ino bang Arduino IDE — cac tab
//      config.h, kinematics.*, gait_engine.*, servo_controller.*,
//      sensor_manager.*, websocket_server.* se tu hien ra ben canh.
//   3. Cai dat board "ESP32 Dev Module" qua Boards Manager
//      (Tools > Board > esp32 by Espressif Systems).
//   4. Cai thu vien qua Library Manager (Tools > Manage Libraries):
//        - "Adafruit PWM Servo Driver Library" (adafruit/Adafruit PWM Servo Driver Library)
//        - "WebSockets" by Markus Sattler (links2004/arduinoWebSockets)
//        - "ArduinoJson" by Benoit Blanchon (ban 7.x)
//   5. Chon dung cong COM, nhan Upload.
//
// SAU KHI NAP:
//   - ESP32 tao WiFi AP ten "Hexapod-Robot" (mat khau "hexapod123")
//   - Ket noi phone/laptop vao WiFi nay, dia chi ESP32 la 192.168.4.1
//   - Mo Serial Monitor (115200 baud) de xem log va dung lenh calib
//     servo truc tiep qua Serial (giong code_test_servo.ino cu):
//       Nhap: <id> <goc 0-180>   vi du: 0 90
//   - App/Postman/websocat ket noi toi ws://192.168.4.1:81 de gui
//     lenh JSON (xem chi tiet protocol trong websocket_server.h)
//
// !!! QUAN TRONG - CAN CALIB TRUOC KHI CHAY GAIT !!!
//   Goc IK tinh ra co the sai chieu/lech tam so voi servo thuc te.
//   Truoc khi cho robot di, hay:
//     1. Nang robot len khoi mat dat (chan khong chiu tai)
//     2. Dung lenh Serial "center" de đưa tat ca servo ve 90 do
//     3. Kiem tra tung chan co dung huong "chu Z" nhu anh lap rap
//        goc trong repo hexapod-main (Illustrations/) khong
//     4. Neu chan bi nguoc, dat SERVO_CAL[i].inverted = true trong
//        config.h cho servo do, nap lai
//     5. Neu chan lech tam, chinh SERVO_CAL[i].trim (do) trong config.h
// ============================================================
// HexapodController — Firmware ESP32 cho hexapod 18 servo
// ============================================================

#include <WiFi.h>
#include <Wire.h>
#include "config.h"
#include "servo_controller.h"
#include "kinematics.h"
#include "gait_engine.h"
#include "sensor_manager.h"
#include "websocket_server.h"

ServoController   servoCtrl;
Kinematics        kinematics;
GaitEngine        gaitEngine;
SensorManager     sensors;
HexapodWebSocket  wsServer;

unsigned long _lastStatusBroadcastMs = 0;
unsigned long _lastSensorUpdateMs    = 0;
const unsigned long STATUS_BROADCAST_MS = 500;
const unsigned long SENSOR_UPDATE_MS    = 100;

String _serialBuf = "";

void handleSerialCommand(const String& cmdRaw) {
    String cmd = cmdRaw;
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "center") {
        servoCtrl.centerAll();
        delay(1);
        return;
    }
    if (cmd == "stop") {
        gaitEngine.stop();
        Serial.println("[Serial] Da dung robot");
        return;
    }
    if (cmd == "disable") {
        servoCtrl.disableAll();
        delay(1);
        return;
    }

    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex == -1) {
        Serial.println("Cu phap: <id 0-17> <goc 0-180> | center / stop / disable");
        return;
    }

    int id  = cmd.substring(0, spaceIndex).toInt();
    int ang = cmd.substring(spaceIndex + 1).toInt();

    if (id < 0 || id > 17) {
        Serial.println("Servo ID phai tu 0 den 17");
        return;
    }
    if (ang < 0 || ang > 180) {
        Serial.println("Goc phai tu 0 den 180");
        return;
    }

    float offset = (float)ang - 90.0f;
    servoCtrl.setAngle((uint8_t)id, offset);

    Serial.println("--------------------------------");
    Serial.printf("Servo ID : %d\n", id);
    Serial.printf("Board    : 0x%02X\n", (id < 9) ? PCA9685_ADDR_1 : PCA9685_ADDR_2);
    Serial.printf("Channel  : %d\n", (id < 9) ? id : id - 9);
    Serial.printf("Goc      : %d (offset %.1f)\n", ang, offset);
    Serial.println("--------------------------------");
}

void setup() {
    Serial.begin(DEBUG_BAUD);
    delay(300);

    Serial.println();
    Serial.println("======================================");
    Serial.println("   HEXAPOD CONTROLLER - ESP32 Firmware");
    Serial.println("======================================");

    // I2C init CHI 1 LAN duy nhat tai day
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    delay(5);

    if (!servoCtrl.begin()) {
        Serial.println("[FATAL] Khong khoi tao duoc ServoController!");
    }
    delay(1);

    kinematics.begin();
    delay(1);

    servoCtrl.centerAll();
    delay(10);

    gaitEngine.begin(&kinematics, &servoCtrl);
    delay(1);

    sensors.begin();
    delay(1);

    WiFi.mode(WIFI_AP);
    delay(1);

    bool apOk = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
    if (apOk) {
        Serial.printf("[WiFi] AP '%s' da khoi dong. IP: %s\n",
                      WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("[WiFi] LOI: khong khoi dong duoc Access Point!");
    }
    delay(1);

    wsServer.begin(&gaitEngine, &servoCtrl);
    delay(1);

    Serial.println("======================================");
    Serial.println("Sang san. Ket noi WiFi 'Hexapod-Robot' roi");
    Serial.println("mo WebSocket toi ws://192.168.4.1:81");
    Serial.println("======================================");

    _lastStatusBroadcastMs = millis();
    _lastSensorUpdateMs    = millis();
}

void loop() {
    wsServer.loop();
    gaitEngine.update();

    unsigned long now = millis();

    if (now - _lastSensorUpdateMs >= SENSOR_UPDATE_MS) {
        _lastSensorUpdateMs = now;
        sensors.update();
        if (sensors.isBatteryShutoff()) {
            gaitEngine.stop();
        }
    }

    if (now - _lastStatusBroadcastMs >= STATUS_BROADCAST_MS) {
        _lastStatusBroadcastMs = now;
        bool touch[6];
        sensors.getAllTouch(touch);
        wsServer.broadcastStatus(sensors.getBatteryVoltage(), touch, gaitEngine.getCurrentMode());
    }

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            handleSerialCommand(_serialBuf);
            _serialBuf = "";
        } else if (c != '\r') {
            _serialBuf += c;
            if (_serialBuf.length() > 128) {
                _serialBuf = "";
                Serial.println("[Serial] Lenh qua dai, da xoa buffer.");
            }
        }
    }

    delay(1);
}