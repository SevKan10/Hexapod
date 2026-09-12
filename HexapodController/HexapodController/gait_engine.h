#pragma once
#include <Arduino.h>
#include "config.h"
#include "kinematics.h"
#include "servo_controller.h"

// ============================================================
// GaitEngine — Thuat toan dang di cho hexapod
//
// Ho tro:
//   - Tripod Gait: 3 chan di chuyen cung luc (L1,R2,L3 | R1,L2,R3)
//   - Wave Gait:   tung chan di chuyen lan luot (on dinh nhat)
//
// Input moi tick:
//   vx: toc do tien/lui   (-1.0 .. 1.0)
//   vy: toc do sang ngang (-1.0 .. 1.0)
//   omega: toc do xoay    (-1.0 .. 1.0)
// ============================================================

enum class GaitType {
    TRIPOD = 0,
    WAVE   = 1,
};

class GaitEngine {
public:
    void begin(Kinematics* ik, ServoController* servo);

    // Goi moi tick trong loop() — tu gioi han theo GAIT_UPDATE_MS
    void update();

    void setVelocity(float vx, float vy, float omega);
    void setBodyPose(float height, float pitch, float roll, float yaw);
    void setGaitType(GaitType type);
    void setMode(uint8_t modeIndex);
    void setSpeedFactor(float factor);

    // Dung — dat tat ca chan ve vi tri mac dinh (standing)
    void stop();

    bool isMoving() const;
    GaitType getCurrentGait() const { return _gaitType; }
    uint8_t  getCurrentMode() const { return _modeIndex; }

private:
    Kinematics*      _ik    = nullptr;
    ServoController* _servo = nullptr;

    float _vx = 0, _vy = 0, _omega = 0;
    float _bodyHeight  = -LEG_STANDING_Z; // chieu cao body (duong)
    float _bodyPitch   = 0;
    float _bodyRoll    = 0;
    float _bodyYaw     = 0;
    float _speedFactor = 1.0f;

    GaitType _gaitType  = GaitType::TRIPOD;
    uint8_t  _modeIndex = 0;

    // Pha buoc cua moi chan (0.0 -> 1.0)
    float _legPhase[6] = {0};

    static constexpr float TRIPOD_PHASE_OFFSET[6] = {
        0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f, // L1,L2,L3,R1,R2,R3
    };
    static constexpr float WAVE_PHASE_OFFSET[6] = {
        0.0f, 1.0f/6.0f, 2.0f/6.0f, 3.0f/6.0f, 4.0f/6.0f, 5.0f/6.0f,
    };

    // Vi tri hien tai / trung tam cua dau chan (BODY FRAME, mm)
    Vec3 _footPos[6];
    Vec3 _footCenter[6];

    Vec3 computeFootTarget(uint8_t legIndex);
    Vec3 computeStanceFootPos(uint8_t legIndex, float phaseProgress);
    Vec3 computeSwingFootPos(uint8_t legIndex, float phaseProgress, const Vec3& target);
    Vec3 applyBodyTransform(const Vec3& footBody, uint8_t legIndex);

    float _phaseSpeed = 0.02f;
    float _stepHeight = GAIT_STEP_HEIGHT;

    unsigned long _lastUpdateMs = 0;
};
