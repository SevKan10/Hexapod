#pragma once
#include <Arduino.h>

// ============================================================
// WiFi Access Point — ESP32 tu tao hotspot, phone ket noi truc tiep
// ============================================================
#define WIFI_AP_SSID       "Hexapod-Robot"
#define WIFI_AP_PASSWORD   "hexapod123"   // phai >= 8 ky tu
#define WIFI_AP_CHANNEL    6
#define WIFI_AP_MAX_CONN   4

#define WS_PORT            81   // WebSocket server port

// ============================================================
// I2C pins — DA TEST OK trong code_test_servo.ino
// ============================================================
#define I2C_SDA_PIN        17
#define I2C_SCL_PIN        16

// ============================================================
// PCA9685 — 2 board, MOI board dung 9 channel (0-8) cho 9 servo
// (Sua loi ban dau: ban cu gan ca 18 servo vao 1 board voi channel
//  chay den 17 — vuot qua gioi han 16 channel/board cua PCA9685)
// ============================================================
#define PCA9685_ADDR_1     0x40   // servo index 0-8
#define PCA9685_ADDR_2     0x41   // servo index 9-17
#define PCA9685_FREQ       50     // 50Hz cho servo RC chuan

// Gia tri raw PWM count (0-4095) ung voi 0 do va 180 do
// Giong het code_test_servo.ino da test hoat dong tren robot thuc te
#define SERVO_PWM_MIN_COUNT   102   // ung voi 0 do
#define SERVO_PWM_MAX_COUNT   512   // ung voi 180 do

// ============================================================
// Kinematics — Kich thuoc chan (mm) — port tu chica-config-2040.txt
// ============================================================
#define COXA_LENGTH        43.0f
#define FEMUR_LENGTH       80.0f
#define TIBIA_LENGTH       134.0f

// Goc gan servo (do) — offset khi servo o center (90 do vat ly)
#define COXA_ATTACH_ANGLE  (-8.0f)
#define FEMUR_ATTACH_ANGLE (35.0f)
#define TIBIA_ATTACH_ANGLE (68.0f)

// Khoang cach giua diem goc cac chan (mm)
#define L1_TO_R1           126.0f
#define L1_TO_L3           167.0f
#define L2_TO_R2           163.0f

// Chieu cao noi chan voi khung
#define LEG_CONNECTION_Z   (-10.0f)
// Chieu cao mac dinh khi dung yen
#define LEG_STANDING_Z     (-120.0f)

// ============================================================
// Leg Layout
// ============================================================
#define LEG_RADIUS         220.0f  // mm — ban kinh dat chan
#define CORNER_LEG_ANGLE   55.0f   // do — goc chan truoc/sau

// ============================================================
// Mode Definitions — port tu chica-config-2040.txt
// ============================================================
struct LegMode {
    float legRadius;
    float cornerAngle;
    float elongation;
    float bodyLift;
    float stepLift;
    float touchSensorCorrection;
    float speedFactor;
    float animationFactor;
};

#define MODE_COUNT         6

static const LegMode MODES[MODE_COUNT] = {
    // radius, cornerAngle, elongation, bodyLift, stepLift, tsCorrect, speed, anim
    { 220, 55, 1.15f,  40, 40,  10, 1.0f, 1.0f }, // STANDARD
    { 210, 55, 1.20f,  35, 30,   0, 2.0f, 0.0f }, // RACE
    { 230, 55, 1.15f,  60, 99, 120, 0.6f, 1.0f }, // OFFROAD
    { 220, 55, 1.15f,  40, 40,  10, 1.0f, 1.0f }, // CUSTOM
    { 220, 60, 1.00f,  45, 35,   0, 0.8f, 1.0f }, // QUADRUPED
    { 185, 30, 1.07f, -40, 80,   0, 1.0f, 1.0f }, // BLOCK
};

static const char* MODE_NAMES[MODE_COUNT] = {
    "STANDARD", "RACE", "OFFROAD", "CUSTOM", "QUADRUPED", "BLOCK"
};

// ============================================================
// Servo calibration — 18 servo, thu tu:
// index 0-8:  L11,L12,L13, L21,L22,L23, L31,L32,L33
// index 9-17: R11,R12,R13, R21,R22,R23, R31,R32,R33
// (thu tu nay PHAI khop voi SERVO_MAP trong kinematics.cpp)
//
// Mapping board/channel giong code_test_servo.ino da test:
//   index < 9  -> board 0 (0x40), channel = index
//   index >= 9 -> board 1 (0x41), channel = index - 9
// ============================================================
struct ServoCal {
    uint8_t board;    // 0 = 0x40, 1 = 0x41
    uint8_t channel;  // 0-8
    int16_t trim;     // do bu goc — dieu chinh khi calib thuc te (mac dinh 0)
    bool    inverted; // true neu servo lap nguoc chieu
};

// QUAN TRONG: trim va inverted o day chi la gia tri mac dinh.
// Phai calib lai tren robot thuc te (xem huong dan calib o cuoi file .ino)
static ServoCal SERVO_CAL[18] = {
    // board, channel, trim, inverted
    { 0, 0, 0, false }, //  0 L11 - Coxa
    { 0, 1, 0, false }, //  1 L12 - Femur
    { 0, 2, 0, false }, //  2 L13 - Tibia
    { 0, 3, 0, false }, //  3 L21
    { 0, 4, 0, false }, //  4 L22
    { 0, 5, 0, false }, //  5 L23
    { 0, 6, 0, false }, //  6 L31
    { 0, 7, 0, false }, //  7 L32
    { 0, 8, 0, false }, //  8 L33
    { 1, 0, 0, false }, //  9 R11
    { 1, 1, 0, false }, // 10 R12
    { 1, 2, 0, false }, // 11 R13
    { 1, 3, 0, false }, // 12 R21
    { 1, 4, 0, false }, // 13 R22
    { 1, 5, 0, false }, // 14 R23
    { 1, 6, 0, false }, // 15 R31
    { 1, 7, 0, false }, // 16 R32
    { 1, 8, 0, false }, // 17 R33
};

// ============================================================
// Touch Sensor GPIO Pins
// GPIO 32,33 ho tro pull-up noi. GPIO 34,35,36,39 la input-only
// (khong pull-up noi, module cam bien phai tu cung cap muc logic ro rang)
// ============================================================
#define TOUCH_SENSOR_COUNT  6
static const uint8_t TOUCH_SENSOR_PINS[6] = { 32, 33, 34, 35, 36, 39 };
// Ten: TS_L1, TS_L2, TS_L3, TS_R1, TS_R2, TS_R3
static const bool TOUCH_HIGH_ACTIVE[6] = { true, true, true, true, true, true };

// ============================================================
// Battery Monitor
// Doi tu GPIO34 (trung voi touch sensor) sang GPIO37 de tranh xung dot.
// GPIO37 thuoc ADC1 nen doc on dinh du WiFi dang hoat dong.
// ============================================================
#define BATTERY_ADC_PIN    37
#define BATTERY_CELLS      2      // 2S LiPo
#define BATTERY_WARN_V     6.4f   // Volts
#define BATTERY_SHUTOFF_V  6.0f   // Volts

// ESP32 ADC chi doc toi da ~3.3V, pin 2S LiPo len toi 8.4V khi day
// => BAT BUOC phai co mach chia ap (vi du R1=100k noi tiep R2=47k)
// RATIO = (R1+R2)/R2. Phai do lai bang von ke va calib RATIO cho khop thuc te.
#define BATTERY_VOLTAGE_DIVIDER_RATIO 3.0f

// ============================================================
// Gait Configuration
// ============================================================
#define GAIT_UPDATE_MS     20     // Update loop 50Hz
#define GAIT_STEP_HEIGHT   40.0f  // mm — nang chan len bao nhieu

// ============================================================
// Debug
// ============================================================
#define DEBUG_BAUD         115200
