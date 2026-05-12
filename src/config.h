#pragma once
#include <stdint.h>

// ─── LOGS ────────────────────────────────────────────────────────────────────
// Les macros LOG_E/W/I/D sont définies dans log.h et wrappent Serial.printf.
// Les blocs inactifs sont éliminés à la compilation (condition constante).
//
//   0  →  aucun log — zéro CPU, zéro UART (compétition)
//   1  →  [E] erreurs critiques seulement
//   2  →  [E][W] + obstacles, timeouts
//   3  →  [E][W][I] + positions, état match  ← défaut debug
//   4  →  [E][W][I][D] + chaque startGo(), détail PID
//
// Usage :  LOG_I("TAG", "x=%.0f y=%.0f", x, y);
#define LOG_LEVEL  3

// ─── LIDAR LD06 ───────────────────────────────────────────────────────────────
#define LIDAR_RX_PIN        9
#define LIDAR_BAUD          230400
#define LIDAR_PWM_PIN       8
#define LIDAR_PWM_FREQ_HZ   10000
#define LIDAR_PWM_DUTY      192       // 75% sur 8 bits → ~3600 RPM
#define LIDAR_OFFSET_DEG    0.0f      // décalage fin (à calibrer) — la correction CW + 90° est dans le code

// ─── ENCODEURS AMT102V (quadrature x4, roues codeuses ∅50 mm) ────────────────
#define ENC_LEFT_A_PIN      6
#define ENC_LEFT_B_PIN      7
#define ENC_RIGHT_A_PIN     1
#define ENC_RIGHT_B_PIN     2

#define ENC_PPR             2048
#define ENC_COUNTS_PER_REV  (ENC_PPR * 4)
#define ENC_WHEEL_DIAM_MM   50.58f
#define ENC_WHEELBASE_MM    189.0f  // distance entre les deux roues codeuses
#define MM_PER_COUNT        (3.14159265f * ENC_WHEEL_DIAM_MM / ENC_COUNTS_PER_REV)
#define ENC_LEFT_INVERT     false   // inverser sens encodeur gauche
#define ENC_RIGHT_INVERT    true    // inverser sens encodeur droit

// ─── MOTEURS PAS À PAS (roues de traction ∅60 mm) ────────────────────────────
#define STEPPER_R_STEP_PIN  12
#define STEPPER_R_DIR_PIN   13
#define STEPPER_L_STEP_PIN  11
#define STEPPER_L_DIR_PIN   10
#define STEPPER_EN_PIN      21          // LOW = activé (commun aux deux)
#define STEPPER_L_INVERT    false        // inverser sens logique moteur gauche
#define STEPPER_R_INVERT    true       // inverser sens logique moteur droit

#define STEPPER_STEPS_REV   1600      // 200 pas × 8 micropas
#define DRIVE_WHEEL_DIAM_MM 57.53f
#define STEPS_PER_MM        ((float)STEPPER_STEPS_REV / (3.14159265f * DRIVE_WHEEL_DIAM_MM))

// ─── GÉOMÉTRIE ROBOT ─────────────────────────────────────────────────────────
#define WHEELBASE_MM            145.60f  // distance entre roues motrices (à mesurer)

// ─── DIMENSIONS ROBOT ────────────────────────────────────────────────────────
#define ROBOT_BACK_TO_CENTER_MM  80.9f  // arrière du robot → axe des roues

// ─── PRISE DE STOCK ──────────────────────────────────────────────────────────
#define STOCK_TOOL_OFFSET_MM    210.0f  // distance centre robot → centre stock en position de prise
#define STOCK_STAGING_MM        150.0f  // recul derrière la position de prise avant l'approche finale
#define STOCK_DEPOSE_OFFSET_MM  180.0f  // distance centre robot → centre stock en position de dépose

// Recalage X par poussée stock contre bordure Ouest :
// stock plaqué → face Est du stock à X=150mm → centre robot à X=150+135=285mm
#define STOCK_RECAL_MM          285.0f  // offset centre robot depuis la bordure quand stock plaqué

// ─── CINÉMATIQUE ─────────────────────────────────────────────────────────────
#define DEFAULT_SPEED_MMS   2500.0f    // mm/s
#define DEFAULT_ACCEL_MMS2  2500.0f   // mm/s²  — freinage en 320 mm depuis 800 mm/s
#define TURN_SPEED_MMS      2500.0f
#define TURN_ACCEL_MMS2     2500.0f

// ─── TABLE DE JEU ────────────────────────────────────────────────────────────
#define TABLE_WIDTH_MM      3000.0f   // axe X (horizontal)
#define TABLE_HEIGHT_MM     2000.0f   // axe Y (vertical, vers le bas)
#define TABLE_MARGIN_MM     100.0f    // épaisseur marge bord (filtre points LIDAR = murs)

// ─── ORIENTATIONS CARDINALES (repère table : 0°=Est, 90°=Nord, sens positif=CCW)
#define ANGLE_NORTH         90.0f
#define ANGLE_EAST           0.0f
#define ANGLE_SOUTH        270.0f
#define ANGLE_WEST         180.0f

// ─── DÉTECTION OBSTACLE ──────────────────────────────────────────────────────
#define LIDAR_BODY_DIST_MM  80.0f     // ignore points < 80 mm (intérieur du robot)

// ─── ZONES AVEUGLES LIDAR (poteaux structurels) ───────────────────────────────
// Angles en repère robot : 0°=avant, 90°=gauche, 180°=arrière, 270°=droite
// Calibrer en observant les angles des poteaux sur Teleplot (pid_dL/dR ou lidar)
#define LIDAR_BLIND_L_START  75.0f    // début zone aveugle poteau gauche (°)
#define LIDAR_BLIND_L_END   105.0f    // fin   zone aveugle poteau gauche (°)
#define LIDAR_BLIND_R_START 255.0f    // début zone aveugle poteau droite (°)
#define LIDAR_BLIND_R_END   285.0f    // fin   zone aveugle poteau droite (°)

#define OBS_DETECT_DIST_MM  400.0f    // distance de détection devant/derrière
#define OBS_WIDTH_MM        200.0f    // largeur zone de détection (≥ largeur robot)
#define OBS_MIN_DIST_MM     60.0f     // distance projetée min dans la zone obstacle
#define OBS_CONFIDENCE_MIN  100       // seuil confiance point LIDAR
#define OBS_WAIT_MS          3000     // timeout attente dégagement adversaire (ms)
#define OBS_POLL_MS            20     // période vérification obstacle pendant go()
#define OBS_BACKUP_MM       100.0f    // recul après détection avant attente
#define OBS_STOP_ACCEL_MMS2 2000.0f   // décélération d'urgence (freinage agressif)
#define LIDAR_LED_DIST_MM   OBS_DETECT_DIST_MM  // seuil LEDs = seuil détection robot

// ─── ASSERVISSEMENT EN POSITION — double PID indépendant (roue D / roue G) ────
//
//  Chaque roue a son propre PID : consigne_position → erreur → PID → vitesse moteur.
//  Les deux boucles sont indépendantes : elles se synchronisent naturellement
//  (la roue en retard reçoit une consigne de vitesse plus élevée).
//
//  RÉGLAGE — procédure recommandée :
//    1. KI=0, KD=0. Augmenter KP jusqu'à décelération nette sans dépassement.
//       Valeur de départ : KP = savedAccel / savedSpeed (≈ 0.5 avec les défauts).
//    2. Si le robot s'arrête systématiquement avant la cible (friction statique) :
//       augmenter KI prudemment. Surveiller I_MAX.
//    3. Si oscillations en fin de course : augmenter KD.
//
//  Symptômes → actions :
//    Dépassement          → baisser KP
//    Undershoot / calage  → augmenter KI ou MIN_SPD
//    Oscillations finales → augmenter KD
//
#define ENC_P1_KP         2.0f    // proportionnel : mm/s par mm d'erreur position
#define ENC_P1_KI         0.01f    // intégral      : mm/s par (mm · s) — anti-trainage
#define ENC_P1_KD         0.2f   // dérivé        : amortissement en fin de course
#define ENC_P1_I_MAX     50.0f    // anti-windup   : saturation intégrale (mm/s)
#define ENC_P1_STOP_MM    5.0f    // seuil d'arrêt translation : erreur avg < valeur → stop (mm)
#define ENC_P1_STOP_DEG   2.0f    // seuil d'arrêt rotation    : erreur avg < valeur → stop (°)
#define ENC_P1_MIN_SPD    8.0f    // vitesse min   : en dessous → stopMove() (mm/s)

// ─── CHRONO DE MATCH ─────────────────────────────────────────────────────────
#define MATCH_DURATION_MS  100000UL   // durée totale du match (100 s)
#define MATCH_ENDGAME_MS    80000UL   // déclenchement repli fin de match (80 s)

// ─── ACTIONNEURS I2C ─────────────────────────────────────────────────────────
#define PCA9685_I2C_ADDR    0x40   // adresse PCA9685 (A0-A5 = GND)
#define PCF8574_I2C_ADDR    0x20   // adresse PCF8574 (A0-A2 = GND)

// ─── ÉCRAN SSD1306 I2C ───────────────────────────────────────────────────────
#define DISPLAY_SCL_PIN     4
#define DISPLAY_SDA_PIN     5
#define DISPLAY_UPDATE_MS   500       // 2 Hz

// ─── TÂCHES FREERTOS ─────────────────────────────────────────────────────────
#define TASK_LIDAR_CORE     0
#define TASK_LIDAR_PRIO     2
#define TASK_LIDAR_STACK    4096

#define TASK_STRATEGY_CORE  1
#define TASK_STRATEGY_PRIO  2
#define TASK_STRATEGY_STACK 4096

#define TASK_DISPLAY_CORE   1
#define TASK_DISPLAY_PRIO   1
#define TASK_DISPLAY_STACK  3072

// ─── BOUTONS ─────────────────────────────────────────────────────────────────
#define BTN_TIRETTE_PIN     14    // pull-up externe — LOW = tirette en place
#define BTN_TEAM_PIN        17    // pull-up interne — LOW = pressé
#define BTN_INIT_PIN         3    // pull-up interne — LOW = pressé

// ─── NEOPIXEL ────────────────────────────────────────────────────────────────
#define NEO_PIN             46
#define NEO_COUNT            7

// ─── ÉQUIPE ──────────────────────────────────────────────────────────────────
enum class Team : uint8_t { YELLOW, BLUE };
