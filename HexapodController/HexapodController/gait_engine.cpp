#include "gait_engine.h"

// Dinh nghia static constexpr array (can o .cpp de link dung tren mot so trinh bien dich)
constexpr float GaitEngine::TRIPOD_PHASE_OFFSET[6];
constexpr float GaitEngine::WAVE_PHASE_OFFSET[6];

void GaitEngine::begin(Kinematics* ik, ServoController* servo) {
    _ik    = ik;
    _servo = servo;

    for (uint8_t i = 0; i < 6; i++) {
        _footPos[i]    = _ik->getDefaultFootPos(i); // body frame
        _footCenter[i] = _footPos[i];
        _legPhase[i]   = TRIPOD_PHASE_OFFSET[i];
    }

    _lastUpdateMs = millis();
    Serial.println("[GaitEngine] Da khoi tao");
}

void GaitEngine::setVelocity(float vx, float vy, float omega) {
    _vx    = constrain(vx,    -1.0f, 1.0f);
    _vy    = constrain(vy,    -1.0f, 1.0f);
    _omega = constrain(omega, -1.0f, 1.0f);
}

void GaitEngine::setBodyPose(float height, float pitch, float roll, float yaw) {
    _bodyHeight = constrain(height, 60.0f, 160.0f);
    _bodyPitch  = constrain(pitch,  -20.0f, 20.0f);
    _bodyRoll   = constrain(roll,   -20.0f, 20.0f);
    _bodyYaw    = constrain(yaw,    -30.0f, 30.0f);
}

void GaitEngine::setGaitType(GaitType type) {
    _gaitType = type;
    for (uint8_t i = 0; i < 6; i++) {
        _legPhase[i] = (type == GaitType::TRIPOD) ? TRIPOD_PHASE_OFFSET[i] : WAVE_PHASE_OFFSET[i];
    }
}

void GaitEngine::setMode(uint8_t modeIndex) {
    if (modeIndex >= MODE_COUNT) return;
    _modeIndex   = modeIndex;
    _speedFactor = MODES[modeIndex].speedFactor;
    _stepHeight  = MODES[modeIndex].stepLift;
}

void GaitEngine::setSpeedFactor(float factor) {
    _speedFactor = constrain(factor, 0.1f, 3.0f);
}

bool GaitEngine::isMoving() const {
    return fabsf(_vx) > 0.05f || fabsf(_vy) > 0.05f || fabsf(_omega) > 0.05f;
}

void GaitEngine::stop() {
    _vx = _vy = _omega = 0;
    for (uint8_t i = 0; i < 6; i++) {
        _footPos[i]    = _ik->getDefaultFootPos(i);
        _footCenter[i] = _footPos[i];
        LegAngles angles = _ik->solveIK(i, _footPos[i]);
        if (angles.valid) _ik->applyAngles(i, angles, *_servo);
    }
}

// ============================================================
// Vi tri trung tam moi chan (body frame), dich chuyen theo velocity
// ============================================================
Vec3 GaitEngine::computeFootTarget(uint8_t legIndex) {
    const LegOrigin& origin = _ik->getLegOrigin(legIndex);

    float maxStep = 80.0f * _speedFactor;
    float dx = _vx * maxStep;
    float dy = _vy * maxStep;

    float legAngleRad = origin.defaultAngle * DEG_TO_RAD;
    float rx = -sinf(legAngleRad) * _omega * 40.0f;
    float ry =  cosf(legAngleRad) * _omega * 40.0f;

    Vec3 defaultPos = _ik->getDefaultFootPos(legIndex); // body frame
    return Vec3(defaultPos.x + dx + rx,
                defaultPos.y + dy + ry,
                origin.position.z - _bodyHeight + (-LEG_CONNECTION_Z));
}

// ============================================================
// Stance phase: chan tren dat, di nguoc voi huong body de day body toi
// ============================================================
Vec3 GaitEngine::computeStanceFootPos(uint8_t legIndex, float phaseProgress) {
    (void)phaseProgress;
    Vec3 target = computeFootTarget(legIndex);
    float maxStep = 80.0f * _speedFactor;

    return Vec3(
        _footPos[legIndex].x + (-_vx * maxStep * 0.04f),
        _footPos[legIndex].y + (-_vy * maxStep * 0.04f),
        target.z
    );
}

// ============================================================
// Swing phase: chan tren khong, di den target theo hinh parabola
// ============================================================
Vec3 GaitEngine::computeSwingFootPos(uint8_t legIndex, float phaseProgress, const Vec3& target) {
    const Vec3& from = _footCenter[legIndex];
    float t = phaseProgress;

    float liftHeight = _stepHeight * 4.0f * t * (1.0f - t);

    return Vec3(
        from.x + (target.x - from.x) * t,
        from.y + (target.y - from.y) * t,
        target.z + liftHeight
    );
}

// ============================================================
// Update Loop — goi moi GAIT_UPDATE_MS ms
// ============================================================
void GaitEngine::update() {
    unsigned long now = millis();
    if (now - _lastUpdateMs < GAIT_UPDATE_MS) return;
    float dt = (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    const float* phaseOffsets = (_gaitType == GaitType::TRIPOD) ? TRIPOD_PHASE_OFFSET : WAVE_PHASE_OFFSET;
    (void)phaseOffsets;

    float speed = (fabsf(_vx) + fabsf(_vy) + fabsf(_omega)) / 3.0f;
    float deltaPhase = speed * _speedFactor * _phaseSpeed * 60.0f * dt;

    for (uint8_t i = 0; i < 6; i++) {
        if (isMoving()) {
            _legPhase[i] += deltaPhase;
            if (_legPhase[i] >= 1.0f) _legPhase[i] -= 1.0f;
        }

        Vec3 newFootPos;
        if (_legPhase[i] < 0.5f) {
            float swingProgress = _legPhase[i] / 0.5f;
            Vec3 target = computeFootTarget(i);
            newFootPos = computeSwingFootPos(i, swingProgress, target);
        } else {
            float stanceProgress = (_legPhase[i] - 0.5f) / 0.5f;
            newFootPos = computeStanceFootPos(i, stanceProgress);
            _footCenter[i] = newFootPos;
        }

        Vec3 transformedPos = applyBodyTransform(newFootPos, i); // van la body frame
        _footPos[i] = newFootPos;

        LegAngles angles = _ik->solveIK(i, transformedPos);
        if (angles.valid) {
            _ik->applyAngles(i, angles, *_servo);
        }
    }
}

// ============================================================
// Ap dung nghieng body (pitch/roll), xap xi goc nho.
// Input/Output deu la BODY FRAME (khong phai leg-relative),
// vi Kinematics::solveIK tu tru origin.position ben trong.
// ============================================================
Vec3 GaitEngine::applyBodyTransform(const Vec3& footBody, uint8_t legIndex) {
    const LegOrigin& origin = _ik->getLegOrigin(legIndex);

    // Chuyen tam ve goc chan de tinh xoay, roi cong lai origin.position
    Vec3 rel = footBody - origin.position;

    float rollRad  = _bodyRoll  * DEG_TO_RAD;
    float pitchRad = _bodyPitch * DEG_TO_RAD;

    float dy       = -rel.z * sinf(rollRad);
    float dz_roll  =  rel.y * sinf(rollRad);
    float dz_pitch =  rel.x * sinf(pitchRad);

    Vec3 rotated(rel.x, rel.y + dy, rel.z + dz_roll + dz_pitch);
    return rotated + origin.position; // tra ve body frame
}
