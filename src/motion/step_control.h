#pragma once
#include <FastAccelStepper.h>
#include "../config.h"

// Contrôle pur en pas avec dead-reckoning de la pose.
// La pose est calculée depuis les pas commandés (pas d'encodeurs dans la boucle).
class StepControl {
public:
    StepControl();
    bool begin();

    // ── Déplacements bloquants ────────────────────────────────────────────────
    void go(float mm);              // relatif, positif = avant
    void turn(float deg);           // relatif, positif = gauche (CCW)

    // ── Déplacements non-bloquants ────────────────────────────────────────────
    void startGo(float mm);
    void startRunOpen(float mm);            // vitesse continue, arrêt géré par l'appelant
    void startTurnOpen(float deg);          // rotation continue (roues opposées), arrêt géré par l'appelant
    void setMotorSpeeds(float leftMmS, float rightMmS);      // magnitude seule (direction inchangée)
    void setMotorVelocities(float leftMmS, float rightMmS);  // vitesse signée, gère la direction
    void startTurn(float deg);
    bool isMoving() const;
    void stop();                            // arrêt immédiat (force)
    void softStop(float accelOverride = 0); // arrêt avec décélération
    void disableMotors();
    void enableMotors();

    // ── Mise à jour pose depuis les pas ──────────────────────────────────────
    void syncPose();

    // ── Pose ─────────────────────────────────────────────────────────────────
    void  setPosition(float x_mm, float y_mm, float theta_deg);
    float getX()        const { return _x; }
    float getY()        const { return _y; }
    float getTheta()    const { return _theta; }
    float getThetaDeg() const { return _theta * 57.2957795f; }

    // ── Cinématique ───────────────────────────────────────────────────────────
    void  setSpeed(float mmS);
    void  setAcceleration(float mmS2);
    void  pushAcceleration();   // applique _accel sur les steppers (après softStop)
    float getSpeed()        const { return _speed; }
    float getAcceleration() const { return _accel; }

private:
    FastAccelStepperEngine _engine;
    FastAccelStepper *_stepL = nullptr;
    FastAccelStepper *_stepR = nullptr;

    float   _x = 0, _y = 0, _theta = 0;
    float   _speed = DEFAULT_SPEED_MMS;
    float   _accel = DEFAULT_ACCEL_MMS2;
    int32_t _refL = 0, _refR = 0;

    void     _applySpeed();
    void     _waitDone();
    int32_t  _mmToSteps(float mm)  const;
    uint32_t _mmSToHz(float mmS)   const;
    void     _updatePoseFromDelta(float leftMm, float rightMm);
};
