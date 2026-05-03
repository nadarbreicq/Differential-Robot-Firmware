// ═══════════════════════════════════════════════════════════════════════════════
//  ACTIONNEURS  —  définir ici les instances et les séquences
// ═══════════════════════════════════════════════════════════════════════════════

#include "actuators.h"
#include <Arduino.h>

// ─── Instances matériel ──────────────────────────────────────────────────────

PCA9685 pca(PCA9685_I2C_ADDR);
PCF8574 pcf(PCF8574_I2C_ADDR);

// ─── Servomoteurs ─────────────────────────────────────────────────────────────
// {canal, minUs, maxUs, minDeg, maxDeg}
Servo servoBras(pca, {0, 500, 2500, 0, 180});   // canal 0, course 0°-180°

// ─── Init ────────────────────────────────────────────────────────────────────

bool actuatorsInit() {
    bool ok = true;
    ok &= pca.begin(50.0f);   // 50 Hz standard servo
    ok &= pcf.begin(0xFF);    // toutes sorties HIGH (repos)
    return ok;
}

void actuatorsDisable() {
    servoBras.detach();        // coupe le signal PWM → servo libre
    pcf.writeByte(0xFF);       // toutes sorties PCF8574 au repos (HIGH)
}

// ─── Séquences ───────────────────────────────────────────────────────────────

void deployerBras() {
    servoBras.moveTo(150, 60);   // 150° à 60°/s — position déployée
}

void retracteBras() {
    servoBras.moveTo(10, 60);    // 10° à 60°/s — position rétractée
}
