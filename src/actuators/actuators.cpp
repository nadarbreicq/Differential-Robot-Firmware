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
Servo servoBrasGauche(pca, {1, 500, 2500, 0, 180});   // canal 1, course 0°-180°
Servo servoLifter(pca, {2, 2000, 1000, 0, 180});     // canal 2, course 0°-180°
Servo servoGripper(pca, {3, 2500, 500, 0, 180});    // canal 3, course 0°-180° 

// ─── Init ────────────────────────────────────────────────────────────────────

bool actuatorsInit() {
    bool ok = true;
    ok &= pca.begin(50.0f);   // 50 Hz standard servo
    ok &= pcf.begin(0xFF);    // toutes sorties HIGH (repos)
    return ok;
}

void actuatorsDisable() {
    servoBrasDroit.detach();
    servoBrasGauche.detach();
    servoLifter.detach();
    servoGripper.detach();
    pcf.writeByte(0xFF);       // toutes sorties PCF8574 au repos (HIGH)
}

// ─── Séquences ───────────────────────────────────────────────────────────────

void initActuators() {
    retracteBrasDroit();
    retracteBrasGauche();
    retracterLifter();
    fermerGripper();
}

void deployerBrasDroit(int speed) {
    servoBrasDroit.moveToPercent(78, speed);
}

void retracteBrasDroit(int speed) {
    servoBrasDroit.moveToPercent(48, speed);
}

void deployerBrasGauche(int speed) {
    servoBrasGauche.moveToPercent(80, speed);
}

void retracteBrasGauche(int speed) {
    servoBrasGauche.moveToPercent(50, speed);
}

void deployerLifter(int speed) {
    servoLifter.moveToPercent(90, speed);
}

void retracterLifter(int speed) {
    servoLifter.moveToPercent(14, speed);
}

void ouvrirGripper(int speed) {
    servoGripper.moveToPercent(35, speed);
}

void libererStock(int speed) {
    servoGripper.moveToPercent(10, speed);
}

void fermerGripper(int speed) {
    servoGripper.moveToPercent(0, speed);
}

void fastFermerGripper(){
    servoGripper.setAngle(0);
}

void sequencePrise() {
    ouvrirGripper();
    deployerLifter();
    fermerGripper();
}
