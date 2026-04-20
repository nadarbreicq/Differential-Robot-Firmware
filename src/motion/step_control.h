#pragma once
#include <FastAccelStepper.h>
#include "../config.h"

// Contrôle pur en pas avec dead-reckoning de la pose.
// Pas d'encodeurs : la pose est calculée depuis les pas commandés.
class StepControl {
public:
    StepControl();
    bool begin();

    // ── Déplacements bloquants ────────────────────────────────────────────────
    void go(float mm);              // relatif, positif = avant
    void turn(float deg);           // relatif, positif = gauche (CCW)

    // ── Déplacements non-bloquants (pour surveillance obstacle) ───────────────
    void startGo(float mm);
    void startTurn(float deg);
    bool isMoving() const;
    void stop();

    // ── Mise à jour pose depuis les pas actuels ───────────────────────────────
    // À appeler après stop() ou en cours de mouvement pour garder la pose à jour
    void syncPose();

    // ── Pose ─────────────────────────────────────────────────────────────────
    void  setPosition(float x_mm, float y_mm, float theta_deg);
    float getX()        const { return _x; }
    float getY()        const { return _y; }
    float getTheta()    const { return _theta; }         // radians
    float getThetaDeg() const { return _theta * 57.2957795f; }

    // ── Cinématique ───────────────────────────────────────────────────────────
    void setSpeed(float mmS);
    void setAcceleration(float mmS2);

private:
    FastAccelStepperEngine _engine;
    FastAccelStepper *_stepL = nullptr;
    FastAccelStepper *_stepR = nullptr;

    float   _x = 0, _y = 0, _theta = 0;
    float   _speed = DEFAULT_SPEED_MMS;
    float   _accel = DEFAULT_ACCEL_MMS2;

    // Référence en pas pour syncPose()
    int32_t _refL = 0, _refR = 0;

    void     _applySpeed();
    void     _waitDone();
    int32_t  _mmToSteps(float mm)  const;
    uint32_t _mmSToHz(float mmS)   const;
    void     _updatePoseFromDelta(float leftMm, float rightMm);
};
