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
Servo servoBrasDroit(pca, {0, 2500, 500, 0, 180});   // canal 0, course 0°-180°
Servo servoBrasGauche(pca, {1, 500, 2500, 0, 180});   // canal 0, course 0°-180°

// ─── Init ────────────────────────────────────────────────────────────────────

bool actuatorsInit() {
    bool ok = true;
    ok &= pca.begin(50.0f);   // 50 Hz standard servo
    ok &= pcf.begin(0xFF);    // toutes sorties HIGH (repos)
    return ok;
}

void actuatorsDisable() {
    servoBrasDroit.detach();        // coupe le signal PWM → servo libre
    servoBrasGauche.detach();
    pcf.writeByte(0xFF);       // toutes sorties PCF8574 au repos (HIGH)
}

// ─── Séquences ───────────────────────────────────────────────────────────────

void initActuators() {
    servoBrasDroit.setPercent(48);
    servoBrasGauche.setPercent(50);
}

void deployerBrasDroit() {
    servoBrasDroit.moveToPercent(78, 60);
}

void retracteBrasDroit() {
    servoBrasGauche.moveToPercent(48, 60);
}

void deployerBrasGauche() {
    servoBrasGauche.moveToPercent(80, 60);
}

void retracteBrasGauche() {
    servoBrasGauche.moveToPercent(50, 60);
}
