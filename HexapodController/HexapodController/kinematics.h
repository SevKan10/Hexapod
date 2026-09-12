#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"

// Forward declare de tranh include vong (servo_controller.h se include kinematics? Khong, chi 1 chieu)
class ServoController;

// ============================================================
// Kinematics — Tinh Inverse Kinematics cho hexapod
//
// He toa do:
//   X: truoc/sau (duong = truoc)
//   Y: trai/phai (duong = phai)
//   Z: len/xuong (duong = len)
//
// Moi chan co 3 DOF:
//   Coxa  (khop 1): xoay ngang (yaw)
//   Femur (khop 2): nang/ha phan tren
//   Tibia (khop 3): gap phan duoi
// ============================================================

struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    float length() const { return sqrtf(x * x + y * y + z * z); }
};

// Ket qua IK cho 1 chan (3 goc, don vi: do, la OFFSET so voi tam servo,
// tuc gia tri 0 = servo dung o vi tri center 90 do vat ly)
struct LegAngles {
    float coxa;
    float femur;
    float tibia;
    bool  valid;
};

// Vi tri goc cua moi chan tren khung robot (body frame)
struct LegOrigin {
    Vec3  position;      // vi tri diem gan chan (mm)
    float defaultAngle;  // goc coxa mac dinh (do)
};

class Kinematics {
public:
    void begin();

    // Tinh IK cho 1 chan
    // legIndex: 0-5 (L1,L2,L3,R1,R2,R3)
    // footTarget: toa do dau chan trong body frame (relative to body center)
    LegAngles solveIK(uint8_t legIndex, const Vec3& footTarget);

    // Ap dung goc vao servo (qua ServoController)
    void applyAngles(uint8_t legIndex, const LegAngles& angles, ServoController& servo);

    LegOrigin getLegOrigin(uint8_t legIndex) const { return _legOrigins[legIndex]; }

    // Vi tri foot target mac dinh (standing position), trong body frame
    Vec3 getDefaultFootPos(uint8_t legIndex) const;

    // Mapping: legIndex -> servo index [coxa, femur, tibia]
    static const uint8_t SERVO_MAP[6][3];

private:
    LegOrigin _legOrigins[6];
};
