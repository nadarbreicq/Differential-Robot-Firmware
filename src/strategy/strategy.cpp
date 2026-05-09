// ═══════════════════════════════════════════════════════════════════════════════
//  STRATÉGIE ROBOT  —  modifier ce fichier pour définir le comportement
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Commandes disponibles :
//
//  robot.go(mm)                         avance (>0) ou recule (<0)
//  robot.turn(deg)                      tourne à gauche (>0) ou droite (<0)
//  robot.gotoXY(x, y)                   va en (x,y) mm — angle d'arrivée libre
//  robot.gotoXY(x, y, angle_deg)        idem avec orientation finale
//
//  robot.setPosition(x, y, theta_deg)   recalage / datum / position initiale
//  robot.setSpeed(mm_s)                 change la vitesse de déplacement
//
//  robot.disableObstacle()              désactive détection adversaire
//  robot.enableObstacle()               réactive détection adversaire
//
//  Repère table : origine coin haut-gauche
//                X+ = droite (3000 mm), Y+ = bas (2000 mm)
//                angle 0° = droite (+X), 90° = haut (-Y), sens positif = anti-horaire
//  Table  : 3000 mm × 2000 mm
// ═══════════════════════════════════════════════════════════════════════════════

#include "strategy.h"
#include "../actuators/actuators.h"
#include "../utils.h"

// ─── Calibration géométrique ─────────────────────────────────────────────────
void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR) {
    robot.enableMotors();
    robot.disableObstacle();

    Serial.println("\n=== CALIBRATION ANGLE (turn 360) ===");
    Serial.printf("WHEELBASE_MM actuel : %.2f\n", (double)WHEELBASE_MM);

    int32_t l0 = encL.getCount(), r0 = encR.getCount();

    robot.turn(720);
    wait(200);

    float arcL = (float)(encL.getCount() - l0) * MM_PER_COUNT;
    float arcR = (float)(encR.getCount() - r0) * MM_PER_COUNT;

    float actualAngle  = (arcR - arcL) / ENC_WHEELBASE_MM * (180.0f / 3.14159265f);
    float newWheelbase = WHEELBASE_MM * (720.0f / actualAngle);

    Serial.printf("arcG=%.1fmm  arcD=%.1fmm  angle_reel=%.1f deg\n",
                  (double)arcL, (double)arcR, (double)actualAngle);

    Serial.println("\n=== VALEUR A COPIER DANS config.h ===");
    Serial.printf("#define WHEELBASE_MM  %.2ff\n", (double)newWheelbase);
    Serial.println("=====================================\n");
}

// ─── Prise de stock ───────────────────────────────────────────────────────────
//
//  angleDeg : direction d'approche (0°=droite, 90°=haut/-Y, 180°=gauche, 270°=bas/+Y)
//
//  Géométrie (repère table, Y+ vers le bas) :
//
//    [staging] ──STOCK_STAGING_MM──▶ [prise] ──STOCK_TOOL_OFFSET_MM──▶ [stock]
//
//  Le robot navigue en staging, s'oriente, puis avance en ligne droite.
//
void takeStock(Robot &robot, float x, float y, float angleDeg) {
    float ar    = angleDeg * (3.14159265f / 180.0f);
    float cos_a = cosf(ar);
    float sin_a = sinf(ar);

    // Position de prise : centre robot à STOCK_TOOL_OFFSET_MM en retrait du stock
    float pickX = x - STOCK_TOOL_OFFSET_MM * cos_a;
    float pickY = y + STOCK_TOOL_OFFSET_MM * sin_a;

    // Position de staging : encore plus en retrait
    float stageX = pickX - STOCK_STAGING_MM * cos_a;
    float stageY = pickY + STOCK_STAGING_MM * sin_a;

    // 1. Aller en staging (angle d'arrivée libre — navigation optimale)
    robot.gotoXY(stageX, stageY);

    // 2. S'orienter face au stock depuis le staging
    robot.gotoXY(stageX, stageY, angleDeg);

    // 3. Avancer en ligne droite vers la position de prise
    robot.go(STOCK_STAGING_MM);

    // 4. Prendre l'élément
    sequencePrise();
}

void deposeStock(Robot &robot, float x, float y, float angleDeg) {
    float ar    = angleDeg * (3.14159265f / 180.0f);
    float cos_a = cosf(ar);
    float sin_a = sinf(ar);

    // Centre robot à STOCK_DEPOSE_OFFSET_MM en retrait du centre de dépose
    float depX = x - STOCK_DEPOSE_OFFSET_MM * cos_a;
    float depY = y + STOCK_DEPOSE_OFFSET_MM * sin_a;

    robot.gotoXY(depX, depY, angleDeg);
    ouvrirGripper();
    wait(1000);
    robot.go(-100);
    wait(1000);
    fermerGripper();
}

// ─── Calage bordure jaune ─────────────────────────────────────────────────────
void runInitYellow(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();
    robot.go(-20);
    // TODO: plaquer contre les deux bordures côté jaune et enregistrer la pose réelle
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 1000, 0);
}

// ─── Calage bordure bleu ──────────────────────────────────────────────────────
void runInitBlue(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();
    robot.go(-20);
    // TODO: plaquer contre les deux bordures côté bleu (symétrique jaune)
    robot.setPosition(TABLE_WIDTH_MM / 2, TABLE_HEIGHT_MM / 2, 0);
}

// ─── Stratégie jaune ──────────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot) {
    robot.enableObstacle();

    robot.go(250);
    robot.gotoXY(500, 1500);
    takeStock(robot, 175, 1600, 180);

    robot.gotoXY(500, 1000,180);
    deposeStock(robot, 50, 1200, 180);

    robot.disableMotors();
}

// ─── Stratégie bleue ─────────────────────────────────────────────────────────
void runStrategyBlue(Robot &robot) {
    robot.enableObstacle();

    robot.go(500);
    robot.turn(90);

    robot.gotoXY(1000, 500);
    robot.gotoXY(0, 0, 180);

    robot.disableMotors();
}

// ─── Repli fin de match jaune ─────────────────────────────────────────────────
void runNearEndYellow(Robot &robot) {
    robot.disableObstacle();
    // TODO: position de sécurité côté jaune
    robot.disableMotors();
}

// ─── Repli fin de match bleu ──────────────────────────────────────────────────
void runNearEndBlue(Robot &robot) {
    robot.disableObstacle();
    // TODO: position de sécurité côté bleu
    robot.disableMotors();
}
