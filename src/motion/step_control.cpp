#include "step_control.h"
#include "../log.h"
#include <math.h>
#include "driver/gpio.h"

StepControl::StepControl() {}

bool StepControl::begin() {
    _engine.init();

    _stepL = _engine.stepperConnectToPin(STEPPER_L_STEP_PIN);
    _stepR = _engine.stepperConnectToPin(STEPPER_R_STEP_PIN);
    if (!_stepL || !_stepR) { LOG_E("STEP", "stepperConnectToPin failed"); return false; }

    _stepL->setDirectionPin(STEPPER_L_DIR_PIN, STEPPER_L_INVERT);
    _stepR->setDirectionPin(STEPPER_R_DIR_PIN, STEPPER_R_INVERT);
    _stepL->setEnablePin(STEPPER_EN_PIN, true);   // true = LOW actif
    _stepR->setEnablePin(STEPPER_EN_PIN, true);
    _stepL->enableOutputs();
    _stepR->enableOutputs();

    // Drive strength maximale (40 mA) sur tous les pins de commande moteur
    gpio_set_drive_capability((gpio_num_t)STEPPER_L_STEP_PIN, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)STEPPER_L_DIR_PIN,  GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)STEPPER_R_STEP_PIN, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)STEPPER_R_DIR_PIN,  GPIO_DRIVE_CAP_3);

    _applySpeed();
    _refL = _stepL->getCurrentPosition();
    _refR = _stepR->getCurrentPosition();
    return true;
}

// ─── Déplacements bloquants ───────────────────────────────────────────────────

void StepControl::go(float mm) {
    startGo(mm);
    _waitDone();
    syncPose();
}

void StepControl::turn(float deg) {
    startTurn(deg);
    _waitDone();
    syncPose();
}

// ─── Déplacements non-bloquants ───────────────────────────────────────────────

void StepControl::startGo(float mm) {
    _applySpeed();
    _refL = _stepL->getCurrentPosition();
    _refR = _stepR->getCurrentPosition();
    int32_t steps = _mmToSteps(mm);
    LOG_D("STEP", "go %.0fmm -> %ld pas  spd=%lu Hz  acc=%lu stp/s2",
          mm, (long)steps,
          (unsigned long)_mmSToHz(_speed),
          (unsigned long)((uint32_t)(_accel * STEPS_PER_MM)));
    _stepL->move(steps);
    _stepR->move(steps);
}

void StepControl::startTurnOpen(float deg) {
    _stepL->setSpeedInHz(_mmSToHz(TURN_SPEED_MMS));
    _stepR->setSpeedInHz(_mmSToHz(TURN_SPEED_MMS));
    _stepL->setAcceleration((uint32_t)(TURN_ACCEL_MMS2 * STEPS_PER_MM));
    _stepR->setAcceleration((uint32_t)(TURN_ACCEL_MMS2 * STEPS_PER_MM));
    _refL = _stepL->getCurrentPosition();
    _refR = _stepR->getCurrentPosition();
    if (deg >= 0) { _stepL->runBackward(); _stepR->runForward();  }  // CCW
    else           { _stepL->runForward();  _stepR->runBackward(); }  // CW
}

void StepControl::startRunOpen(float mm) {
    _applySpeed();
    _refL = _stepL->getCurrentPosition();
    _refR = _stepR->getCurrentPosition();
    if (mm >= 0) { _stepL->runForward(); _stepR->runForward(); }
    else          { _stepL->runBackward(); _stepR->runBackward(); }
}

void StepControl::setMotorSpeeds(float leftMmS, float rightMmS) {
    _stepL->setSpeedInHz(_mmSToHz(leftMmS));
    _stepR->setSpeedInHz(_mmSToHz(rightMmS));
}

void StepControl::setMotorVelocities(float leftMmS, float rightMmS) {
    auto apply = [](FastAccelStepper *s, float mmS) {
        if (fabsf(mmS) < 0.5f) { s->stopMove(); return; }
        s->setSpeedInHz((uint32_t)(fabsf(mmS) * STEPS_PER_MM));
        if (mmS > 0) s->runForward();
        else         s->runBackward();
    };
    apply(_stepL, leftMmS);
    apply(_stepR, rightMmS);
}


void StepControl::startTurn(float deg) {
    // Arc = (wheelbase × π × |deg|) / 360 pour chaque roue
    float arc = WHEELBASE_MM * 3.14159265f * fabsf(deg) / 360.0f;
    int32_t arcSteps = _mmToSteps(arc);
    int32_t sign = (deg >= 0) ? 1 : -1;   // positif = CCW = gauche

    _stepL->setSpeedInHz(_mmSToHz(TURN_SPEED_MMS));
    _stepR->setSpeedInHz(_mmSToHz(TURN_SPEED_MMS));
    _stepL->setAcceleration((uint32_t)(TURN_ACCEL_MMS2 * STEPS_PER_MM));
    _stepR->setAcceleration((uint32_t)(TURN_ACCEL_MMS2 * STEPS_PER_MM));

    _refL = _stepL->getCurrentPosition();
    _refR = _stepR->getCurrentPosition();
    _stepL->move(-sign * arcSteps);   // gauche recule pour tourner à gauche
    _stepR->move( sign * arcSteps);   // droite avance pour tourner à gauche
}

bool StepControl::isMoving() const {
    return _stepL->isRunning() || _stepR->isRunning();
}

void StepControl::stop() {
    _stepL->forceStopAndNewPosition(_stepL->getCurrentPosition());
    _stepR->forceStopAndNewPosition(_stepR->getCurrentPosition());
}

void StepControl::disableMotors() {
    _stepL->disableOutputs();
    _stepR->disableOutputs();
}

void StepControl::enableMotors() {
    _stepL->enableOutputs();
    _stepR->enableOutputs();
}

void StepControl::softStop(float accelOverride) {
    if (accelOverride > 0) {
        uint32_t a = (uint32_t)(accelOverride * STEPS_PER_MM);
        _stepL->setAcceleration(a);
        _stepR->setAcceleration(a);
    }
    _stepL->stopMove();
    _stepR->stopMove();
    _waitDone();
    syncPose();
}

// ─── Pose ─────────────────────────────────────────────────────────────────────

void StepControl::syncPose() {
    int32_t dL = _stepL->getCurrentPosition() - _refL;
    int32_t dR = _stepR->getCurrentPosition() - _refR;
    _refL = _stepL->getCurrentPosition();
    _refR = _stepR->getCurrentPosition();
    _updatePoseFromDelta((float)dL / STEPS_PER_MM, (float)dR / STEPS_PER_MM);
}

void StepControl::setPosition(float x_mm, float y_mm, float theta_deg) {
    _x     = x_mm;
    _y     = y_mm;
    _theta = theta_deg * 3.14159265f / 180.0f;
    _refL  = _stepL->getCurrentPosition();
    _refR  = _stepR->getCurrentPosition();
}

// ─── Cinématique ──────────────────────────────────────────────────────────────

void StepControl::setSpeed(float mmS)        { _speed = mmS; }
void StepControl::setAcceleration(float mmS2) { _accel = mmS2; }
void StepControl::pushAcceleration() {
    uint32_t a = (uint32_t)(_accel * STEPS_PER_MM);
    _stepL->setAcceleration(a);
    _stepR->setAcceleration(a);
}

// ─── Privé ────────────────────────────────────────────────────────────────────

void StepControl::_applySpeed() {
    _stepL->setSpeedInHz(_mmSToHz(_speed));
    _stepR->setSpeedInHz(_mmSToHz(_speed));
    _stepL->setAcceleration((uint32_t)(_accel * STEPS_PER_MM));
    _stepR->setAcceleration((uint32_t)(_accel * STEPS_PER_MM));
}

void StepControl::_waitDone() {
    while (isMoving()) vTaskDelay(1);
}

int32_t StepControl::_mmToSteps(float mm) const {
    return (int32_t)(mm * STEPS_PER_MM);
}

uint32_t StepControl::_mmSToHz(float mmS) const {
    return (uint32_t)(fabsf(mmS) * STEPS_PER_MM);
}


void StepControl::_updatePoseFromDelta(float leftMm, float rightMm) {
    float dDist  = (rightMm + leftMm) * 0.5f;
    float dTheta = (rightMm - leftMm) / WHEELBASE_MM;
    float mid    = _theta + dTheta * 0.5f;
    _x     += dDist * cosf(mid);
    _y     -= dDist * sinf(mid);   // Y+ vers le bas → sin inversé
    _theta += dTheta;
    while (_theta >  3.14159265f) _theta -= 2.0f * 3.14159265f;
    while (_theta < -3.14159265f) _theta += 2.0f * 3.14159265f;
}
