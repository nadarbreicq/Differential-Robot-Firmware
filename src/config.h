#pragma once
#include <stdint.h>

// ─── LIDAR LD06 ───────────────────────────────────────────────────────────────
#define LIDAR_RX_PIN        9
#define LIDAR_TX_PIN        17
#define LIDAR_BAUD          230400
#define LIDAR_PWM_PIN       8
#define LIDAR_PWM_FREQ_HZ   10000
#define LIDAR_PWM_DUTY      128       // 50% sur 8 bits → ~2400 RPM
#define LIDAR_OFFSET_DEG    0.0f      // décalage angulaire LIDAR/axe robot (à calibrer)

// ─── ENCODEURS AMT102V (quadrature x4, roues codeuses ∅50 mm) ────────────────
#define ENC_LEFT_A_PIN      6
#define ENC_LEFT_B_PIN      7
#define ENC_RIGHT_A_PIN     1
#define ENC_RIGHT_B_PIN     2

#define ENC_PPR             2048
#define ENC_COUNTS_PER_REV  (ENC_PPR * 4)
#define ENC_WHEEL_DIAM_MM   50.0f
#define MM_PER_COUNT        (3.14159265f * ENC_WHEEL_DIAM_MM / ENC_COUNTS_PER_REV)

// ─── MOTEURS PAS À PAS (roues de traction ∅60 mm) ────────────────────────────
#define STEPPER_L_STEP_PIN  12
#define STEPPER_L_DIR_PIN   13
#define STEPPER_R_STEP_PIN  11
#define STEPPER_R_DIR_PIN   10
#define STEPPER_EN_PIN      21        // LOW = activé (commun aux deux)
#define STEPPER_L_INVERT    false     // inverser sens logique moteur gauche
#define STEPPER_R_INVERT    true      // inverser sens logique moteur droit

#define STEPPER_STEPS_REV   1600      // 200 pas × 8 micropas
#define DRIVE_WHEEL_DIAM_MM 60.0f
#define STEPS_PER_MM        ((float)STEPPER_STEPS_REV / (3.14159265f * DRIVE_WHEEL_DIAM_MM))

// ─── GÉOMÉTRIE ROBOT ─────────────────────────────────────────────────────────
#define WHEELBASE_MM        200.0f    // distance entre roues motrices (à mesurer)

// ─── CINÉMATIQUE ─────────────────────────────────────────────────────────────
#define DEFAULT_SPEED_MMS   100.0f    // mm/s  (conservateur pour le premier test)
#define DEFAULT_ACCEL_MMS2  150.0f    // mm/s² (augmenter progressivement si OK)
#define TURN_SPEED_MMS      80.0f
#define TURN_ACCEL_MMS2     120.0f

// ─── TABLE DE JEU ────────────────────────────────────────────────────────────
#define TABLE_WIDTH_MM      2000.0f
#define TABLE_HEIGHT_MM     3000.0f
#define TABLE_MARGIN_MM     80.0f     // épaisseur marge bord (filtre points LIDAR = murs)

// ─── DÉTECTION OBSTACLE ──────────────────────────────────────────────────────
#define OBS_DETECT_DIST_MM  400.0f    // distance de détection devant/derrière
#define OBS_WIDTH_MM        300.0f    // largeur zone de détection (≥ largeur robot)
#define OBS_MIN_DIST_MM     30.0f     // ignore points trop proches (corps du robot)
#define OBS_CONFIDENCE_MIN  100       // seuil confiance point LIDAR
#define OBS_WAIT_MS         3000      // timeout avant action subsidiaire (ms)
#define OBS_POLL_MS         50        // période vérification obstacle pendant go()

// ─── ÉCRAN SSD1306 I2C ───────────────────────────────────────────────────────
#define DISPLAY_SCL_PIN     4
#define DISPLAY_SDA_PIN     5
#define DISPLAY_UPDATE_MS   500       // 2 Hz

// ─── TÂCHES FREERTOS ─────────────────────────────────────────────────────────
#define TASK_LIDAR_CORE     0
#define TASK_LIDAR_PRIO     2
#define TASK_LIDAR_STACK    4096

#define TASK_MOTION_CORE    1
#define TASK_MOTION_PRIO    4         // plus haute prio sur C1
#define TASK_MOTION_STACK   4096
#define MOTION_PERIOD_MS    5         // 200 Hz

#define TASK_STRATEGY_CORE  1
#define TASK_STRATEGY_PRIO  2
#define TASK_STRATEGY_STACK 4096

#define TASK_DISPLAY_CORE   1
#define TASK_DISPLAY_PRIO   1
#define TASK_DISPLAY_STACK  3072
