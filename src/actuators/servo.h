#pragma once
#include "pca9685.h"
#include <math.h>

struct ServoConfig {
    uint8_t  channel;     // canal PCA9685 (0-15)
    uint16_t minUs;       // largeur d'impulsion à l'angle min (µs)
    uint16_t maxUs;       // largeur d'impulsion à l'angle max (µs)
    float    minDeg;      // angle min (degrés)
    float    maxDeg;      // angle max (degrés)
};

class Servo {
public:
    Servo(PCA9685 &pca, ServoConfig cfg);

    // ── Positionnement direct ─────────────────────────────────────────────────
    void setAngle(float deg);             // angle en degrés (clampé dans [min, max])
    void setPercent(float pct);           // 0% = minDeg, 100% = maxDeg

    // ── Positionnement avec vitesse (bloquant) ────────────────────────────────
    void moveTo(float deg, float degPerSec);
    void moveToPercent(float pct, float pctPerSec);

    // ── Accès état ────────────────────────────────────────────────────────────
    float getAngle()   const { return _currentDeg; }
    float getPercent() const;

    // ── Relâchement ──────────────────────────────────────────────────────────
    void detach();   // coupe le signal PWM (servo libre)

private:
    PCA9685    &_pca;
    ServoConfig _cfg;
    float       _currentDeg;   // NAN = non initialisé

    uint16_t _degToUs(float deg) const;
    float    _clamp(float v, float lo, float hi) const;
};
