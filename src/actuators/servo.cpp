#include "servo.h"
#include <Arduino.h>

Servo::Servo(PCA9685 &pca, ServoConfig cfg)
    : _pca(pca), _cfg(cfg), _currentDeg(NAN) {}

// ─── Positionnement direct ────────────────────────────────────────────────────

void Servo::setAngle(float deg) {
    deg = _clamp(deg, _cfg.minDeg, _cfg.maxDeg);
    _pca.setPulseUs(_cfg.channel, _degToUs(deg));
    _currentDeg = deg;
}

void Servo::setPercent(float pct) {
    pct = _clamp(pct, 0.0f, 100.0f);
    setAngle(_cfg.minDeg + pct * (_cfg.maxDeg - _cfg.minDeg) / 100.0f);
}

// ─── Positionnement avec vitesse (bloquant) ───────────────────────────────────

void Servo::moveTo(float targetDeg, float degPerSec) {
    targetDeg = _clamp(targetDeg, _cfg.minDeg, _cfg.maxDeg);

    // Premier appel : positionnement immédiat
    if (isnanf(_currentDeg)) {
        setAngle(targetDeg);
        return;
    }

    if (degPerSec <= 0.0f || fabsf(targetDeg - _currentDeg) < 0.5f) {
        setAngle(targetDeg);
        return;
    }

    // Interpolation par pas de 1°
    float dir  = (targetDeg > _currentDeg) ? 1.0f : -1.0f;
    uint32_t stepMs = (uint32_t)(1000.0f / degPerSec);
    if (stepMs < 1) stepMs = 1;

    while (fabsf(targetDeg - _currentDeg) > 0.5f) {
        float next = _currentDeg + dir;
        if (dir > 0 && next > targetDeg) next = targetDeg;
        if (dir < 0 && next < targetDeg) next = targetDeg;
        setAngle(next);
        vTaskDelay(pdMS_TO_TICKS(stepMs));
    }
}

void Servo::moveToPercent(float pct, float pctPerSec) {
    float rangeDeg = _cfg.maxDeg - _cfg.minDeg;
    moveTo(_cfg.minDeg + _clamp(pct, 0.0f, 100.0f) * rangeDeg / 100.0f,
           pctPerSec * rangeDeg / 100.0f);
}

// ─── Relâchement ──────────────────────────────────────────────────────────────

void Servo::detach() {
    _pca.off(_cfg.channel);
    _currentDeg = NAN;
}

// ─── Accesseurs ───────────────────────────────────────────────────────────────

float Servo::getPercent() const {
    if (isnanf(_currentDeg)) return NAN;
    return (_currentDeg - _cfg.minDeg) * 100.0f / (_cfg.maxDeg - _cfg.minDeg);
}

// ─── Privé ────────────────────────────────────────────────────────────────────

uint16_t Servo::_degToUs(float deg) const {
    float ratio = (deg - _cfg.minDeg) / (_cfg.maxDeg - _cfg.minDeg);
    return (uint16_t)(_cfg.minUs + ratio * (_cfg.maxUs - _cfg.minUs) + 0.5f);
}

float Servo::_clamp(float v, float lo, float hi) const {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
