#pragma once
#include "pca9685.h"
#include "pcf8574.h"
#include "servo.h"
#include "../config.h"
#include "../utils.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  ACTIONNEURS  —  déclarer ici les servos, pompes et séquences
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Drivers matériel (instances globales) :
//    pca  → PCA9685 : 16 canaux PWM pour servomoteurs
//    pcf  → PCF8574 : 8 sorties/entrées numériques (pompes, capteurs...)
//
//  Déclarer un servomoteur :
//    extern Servo monServo;                  // dans actuators.h
//    Servo monServo(pca, {canal, minUs, maxUs, minDeg, maxDeg});  // dans actuators.cpp
//
//  Commandes servo :
//    monServo.setAngle(90);                  // va à 90° immédiatement
//    monServo.setPercent(50);                // va à 50% de la course
//    monServo.moveTo(90, 45);               // va à 90° à 45°/s (bloquant)
//    monServo.moveToPercent(100, 30);       // va à 100% à 30%/s (bloquant)
//    monServo.detach();                      // relâche le servo (plus de signal)
//
//  PCF8574 :
//    pcf.setPin(0, false);                  // active la sortie 0 (LOW)
//    pcf.setPin(0, true);                   // désactive la sortie 0 (HIGH)
//    bool etat = pcf.getPin(3);             // lit l'entrée 3
//
//  Appel dans la stratégie :
//    #include "../actuators/actuators.h"
//    deployerBras();                         // appelle une séquence définie ici
//
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Instances matériel ──────────────────────────────────────────────────────
extern PCA9685 pca;
extern PCF8574 pcf;

// ─── Init / Shutdown ─────────────────────────────────────────────────────────
bool actuatorsInit();
void actuatorsDisable();   // détache tous les servos, remet PCF au repos

// ─── Servomoteurs ─────────────────────────────────────────────────────────────
extern Servo servoBrasDroit;
extern Servo servoBrasGauche;

extern Servo servoLifter;
extern Servo servoGripper;

// ─── Séquences d'actionneurs ──────────────────────────────────────────────────
void initActuators();
void deployerBrasDroit(int speed = 100);
void retracteBrasDroit(int speed = 100);
void deployerBrasGauche(int speed = 100);
void retracteBrasGauche(int speed = 100);
void deployerLifter();
void retracterLifter();
void ouvrirGripper();
void libererStock();
void fermerGripper();
void sequencePrise();
