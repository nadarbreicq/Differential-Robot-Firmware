#pragma once
#include <Arduino.h>
#include <FastAccelStepper.h>
#include "encoder.h"
#include "driver/pcnt.h"
#include "../config.h"

struct Pose {
    float x_mm;      // position X (mm)
    float y_mm;      // position Y (mm)
    float theta_rad; // cap (radians)
};

struct Velocity {
    float left_mms;   // vitesse roue gauche (mm/s), signée
    float right_mms;  // vitesse roue droite (mm/s), signée
};

class MotionController {
public:
    MotionController();

    bool begin();

    // Commandes de vitesse en mm/s (signe = sens)
    void setVelocity(float leftMmS, float rightMmS);
    void stop();

    // Mise à jour odométrie (appeler à intervalle régulier)
    void update();

    Pose     getPose()     const { return _pose; }
    Velocity getVelocity() const { return _vel; }

    // Remet la pose à zéro
    void resetPose() { _pose = {0, 0, 0}; }

private:
    FastAccelStepperEngine _engine;
    FastAccelStepper       *_stepL = nullptr;
    FastAccelStepper       *_stepR = nullptr;

    QuadEncoder _encL;
    QuadEncoder _encR;

    Pose     _pose;
    Velocity _vel;

    int32_t  _prevCountL = 0;
    int32_t  _prevCountR = 0;

    static int32_t mmToSteps(float mm);
    static int32_t mmSToHz(float mmS);
};
