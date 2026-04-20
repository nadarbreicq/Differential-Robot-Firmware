#include "stepper_ctrl.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "MOTION";

MotionController::MotionController() : _pose{0,0,0}, _vel{0,0} {}

bool MotionController::begin() {
    // Encodeurs
    if (!_encL.init(ENC_LEFT_A_PIN,  ENC_LEFT_B_PIN,  PCNT_UNIT_0)) { ESP_LOGE(TAG, "Enc L init failed"); return false; }
    if (!_encR.init(ENC_RIGHT_A_PIN, ENC_RIGHT_B_PIN, PCNT_UNIT_1)) { ESP_LOGE(TAG, "Enc R init failed"); return false; }

    // Moteurs pas à pas
    _engine.init();

    _stepL = _engine.stepperConnectToPin(STEPPER_L_STEP_PIN);
    _stepR = _engine.stepperConnectToPin(STEPPER_R_STEP_PIN);

    if (!_stepL || !_stepR) {
        ESP_LOGE(TAG, "FastAccelStepper connect failed");
        return false;
    }

    _stepL->setDirectionPin(STEPPER_L_DIR_PIN);
    _stepR->setDirectionPin(STEPPER_R_DIR_PIN);

    // Broche enable partagée (LOW = activé)
    _stepL->setEnablePin(STEPPER_EN_PIN, true);
    _stepR->setEnablePin(STEPPER_EN_PIN, true);
    _stepL->enableOutputs();
    _stepR->enableOutputs();

    // Accélération par défaut (modifiable via setAcceleration)
    _stepL->setAcceleration((uint32_t)(STEPS_PER_MM * 300.0f));  // 300 mm/s²
    _stepR->setAcceleration((uint32_t)(STEPS_PER_MM * 300.0f));

    return true;
}

void MotionController::setVelocity(float leftMmS, float rightMmS) {
    _vel = { leftMmS, rightMmS };

    auto applyVel = [](FastAccelStepper *s, float mmS) {
        if (fabsf(mmS) < 0.5f) {
            s->stopMove();
            return;
        }
        uint32_t hz = (uint32_t)(fabsf(mmS) * STEPS_PER_MM);
        s->setSpeedInHz(hz);
        if (mmS > 0) s->runForward();
        else         s->runBackward();
    };

    applyVel(_stepL, leftMmS);
    applyVel(_stepR, rightMmS);
}

void MotionController::stop() {
    _stepL->stopMove();
    _stepR->stopMove();
    _vel = {0, 0};
}

void MotionController::update() {
    _encL.update();
    _encR.update();

    int32_t cntL = _encL.getCount();
    int32_t cntR = _encR.getCount();

    int32_t dL = cntL - _prevCountL;
    int32_t dR = cntR - _prevCountR;
    _prevCountL = cntL;
    _prevCountR = cntR;

    float dLeft  = (float)dL * MM_PER_COUNT;
    float dRight = (float)dR * MM_PER_COUNT;

    // Odométrie différentielle (arc)
    float dDist  = (dRight + dLeft) * 0.5f;
    float dTheta = (dRight - dLeft) / WHEELBASE_MM;

    float midTheta = _pose.theta_rad + dTheta * 0.5f;
    _pose.x_mm     += dDist * cosf(midTheta);
    _pose.y_mm     += dDist * sinf(midTheta);
    _pose.theta_rad += dTheta;

    // Normalise theta entre -π et π
    while (_pose.theta_rad >  (float)M_PI) _pose.theta_rad -= 2.0f * (float)M_PI;
    while (_pose.theta_rad < -(float)M_PI) _pose.theta_rad += 2.0f * (float)M_PI;
}

int32_t MotionController::mmToSteps(float mm) {
    return (int32_t)(mm * STEPS_PER_MM);
}

int32_t MotionController::mmSToHz(float mmS) {
    return (int32_t)(fabsf(mmS) * STEPS_PER_MM);
}
