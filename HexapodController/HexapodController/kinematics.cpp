#include "kinematics.h"
#include "servo_controller.h"

// ============================================================
// Mapping: legIndex -> [coxa_idx, femur_idx, tibia_idx]
// Trung khop voi SERVO_CAL trong config.h:
// 0-2: L11,L12,L13 (L1)   3-5: L21,L22,L23 (L2)   6-8: L31,L32,L33 (L3)
// 9-11: R11,R12,R13 (R1)  12-14: R21,R22,R23 (R2) 15-17: R31,R32,R33 (R3)
// ============================================================
const uint8_t Kinematics::SERVO_MAP[6][3] = {
    {  0,  1,  2 },  // L1
    {  3,  4,  5 },  // L2
    {  6,  7,  8 },  // L3
    {  9, 10, 11 },  // R1
    { 12, 13, 14 },  // R2
    { 15, 16, 17 },  // R3
};

void Kinematics::begin() {
    // Vi tri goc cua 6 chan tren khung robot (body frame, tam la trung tam body)
    // Nhin tu tren: L1=truoc trai, L2=giua trai, L3=sau trai
    //               R1=truoc phai, R2=giua phai, R3=sau phai
    float halfWidth1 = L1_TO_R1 / 2.0f;  // 63mm
    float halfWidth2 = L2_TO_R2 / 2.0f;  // 81.5mm
    float legSpan     = L1_TO_L3 / 2.0f; // 83.5mm

    _legOrigins[0] = { Vec3( legSpan, -halfWidth1, LEG_CONNECTION_Z), (180.0f - CORNER_LEG_ANGLE) }; // L1
    _legOrigins[1] = { Vec3( 0.0f,    -halfWidth2, LEG_CONNECTION_Z), 180.0f };                       // L2
    _legOrigins[2] = { Vec3(-legSpan, -halfWidth1, LEG_CONNECTION_Z), (180.0f + CORNER_LEG_ANGLE) };  // L3
    _legOrigins[3] = { Vec3( legSpan,  halfWidth1, LEG_CONNECTION_Z), CORNER_LEG_ANGLE };              // R1
    _legOrigins[4] = { Vec3( 0.0f,     halfWidth2, LEG_CONNECTION_Z), 0.0f };                          // R2
    _legOrigins[5] = { Vec3(-legSpan,  halfWidth1, LEG_CONNECTION_Z), -CORNER_LEG_ANGLE };             // R3

    Serial.println("[Kinematics] Da khoi tao vi tri goc 6 chan");
}

// ============================================================
// Inverse Kinematics Solver
// Input: footTarget — toa do dau chan trong BODY FRAME (goc la
//        trung tam body, KHONG phai goc chan). Ham nay tu tru di
//        vi tri goc cua chan (legOrigin.position) mot lan duy nhat.
// Output: goc offset 3 khop (do, quanh center 90 do vat ly)
// ============================================================
LegAngles Kinematics::solveIK(uint8_t legIndex, const Vec3& footTarget) {
    LegAngles result = {0, 0, 0, false};
    if (legIndex >= 6) return result;

    const LegOrigin& origin = _legOrigins[legIndex];

    // Chuyen ve leg-local frame (coxa joint la goc toa do)
    Vec3 rel = footTarget - origin.position;

    // --- Buoc 1: Goc Coxa (xoay ngang, mat phang XY) ---
    float coxaAngle = atan2f(rel.y, rel.x) * RAD_TO_DEG;
    coxaAngle -= origin.defaultAngle;
    while (coxaAngle >  180.0f) coxaAngle -= 360.0f;
    while (coxaAngle < -180.0f) coxaAngle += 360.0f;

    // --- Buoc 2: Khoang cach ngang tu coxa den foot ---
    float footDist = sqrtf(rel.x * rel.x + rel.y * rel.y);
    float L = footDist - COXA_LENGTH;   // tru do dai coxa
    float H = rel.z;                    // rel.z da tru san LEG_CONNECTION_Z qua origin

    float reach = sqrtf(L * L + H * H);

    if (reach > (FEMUR_LENGTH + TIBIA_LENGTH) || reach < fabsf(FEMUR_LENGTH - TIBIA_LENGTH)) {
        // Ngoai tam voi — khong giai duoc, giu servo o vi tri cu
        return result;
    }

    // --- Buoc 3: Giai 2-link IK (Femur + Tibia) — law of cosines ---
    float cosTibia = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - reach * reach)
                    / (2.0f * FEMUR_LENGTH * TIBIA_LENGTH);
    cosTibia = constrain(cosTibia, -1.0f, 1.0f);
    float tibiaAngle = 180.0f - (acosf(cosTibia) * RAD_TO_DEG);

    float alpha = atan2f(-H, L) * RAD_TO_DEG; // H duong = len -> goc am
    float cosBeta = (FEMUR_LENGTH * FEMUR_LENGTH + reach * reach - TIBIA_LENGTH * TIBIA_LENGTH)
                   / (2.0f * FEMUR_LENGTH * reach);
    cosBeta = constrain(cosBeta, -1.0f, 1.0f);
    float beta = acosf(cosBeta) * RAD_TO_DEG;
    float femurAngle = -(alpha + beta);

    // Ap dung attach angle offset (huong servo vat ly so voi khop)
    coxaAngle  += COXA_ATTACH_ANGLE;
    femurAngle += FEMUR_ATTACH_ANGLE;
    tibiaAngle += TIBIA_ATTACH_ANGLE;

    result.coxa  = coxaAngle;
    result.femur = femurAngle;
    result.tibia = tibiaAngle;
    result.valid = true;
    return result;
}

void Kinematics::applyAngles(uint8_t legIndex, const LegAngles& angles, ServoController& servo) {
    if (!angles.valid || legIndex >= 6) return;
    servo.setAngle(SERVO_MAP[legIndex][0], angles.coxa);
    servo.setAngle(SERVO_MAP[legIndex][1], angles.femur);
    servo.setAngle(SERVO_MAP[legIndex][2], angles.tibia);
}

Vec3 Kinematics::getDefaultFootPos(uint8_t legIndex) const {
    if (legIndex >= 6) return Vec3(0, 0, LEG_STANDING_Z);
    const LegOrigin& origin = _legOrigins[legIndex];
    float defaultAngleRad = origin.defaultAngle * DEG_TO_RAD;

    // Vi tri mac dinh: mo rong ra ngoai theo goc coxa mac dinh
    float radiusReach = COXA_LENGTH + LEG_RADIUS * 0.6f;
    float x = origin.position.x + cosf(defaultAngleRad) * radiusReach;
    float y = origin.position.y + sinf(defaultAngleRad) * radiusReach;
    float z = LEG_STANDING_Z;

    return Vec3(x, y, z);
}
