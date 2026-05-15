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

    // 1. Aller en staging et s'orienter face au stock
    robot.gotoXYenc(stageX, stageY, angleDeg);

    // 2. Approche + recalage — vitesse réduite, restaurée ensuite
    float savedSpd = robot.getSpeed();
    float savedAcc = robot.getAcceleration();
    robot.setSpeedPct(10, 10);
    if (fabsf(angleDeg - ANGLE_WEST) < 1.0f) {
        robot.goStall(500);
        robot.setPosition(STOCK_RECAL_MM, robot.getY(), ANGLE_WEST);
    } else if (fabsf(angleDeg - ANGLE_EAST) < 1.0f) {
        robot.goStall(500);
        robot.setPosition(TABLE_WIDTH_MM - STOCK_RECAL_MM, robot.getY(), ANGLE_EAST);
    } else {
        robot.goPID(STOCK_STAGING_MM);
    }
    robot.setSpeed(savedSpd);
    robot.setAcceleration(savedAcc);

    // 3. Prendre l'élément
    sequencePrise();
    robot.goPID(-50);
    retracterLifter();
}

void deposeStock(Robot &robot, float x, float y, float angleDeg) {
    float ar    = angleDeg * (3.14159265f / 180.0f);
    float cos_a = cosf(ar);
    float sin_a = sinf(ar);

    // Centre robot à STOCK_DEPOSE_OFFSET_MM en retrait du centre de dépose
    float depX = x - STOCK_DEPOSE_OFFSET_MM * cos_a;
    float depY = y + STOCK_DEPOSE_OFFSET_MM * sin_a;

    robot.gotoXYenc(depX, depY, angleDeg);
    libererStock();
    ouvrirGripper();
    //wait(250);
    robot.goPID(-150);
    //wait(250);
    //fermerGripper();
}

void takeStock (Robot &robot, Vec2 poi, float angleDeg) { takeStock (robot, poi.x, poi.y, angleDeg); }
void deposeStock(Robot &robot, Vec2 poi, float angleDeg) { deposeStock(robot, poi.x, poi.y, angleDeg); }

// ─── Calage bordure jaune ─────────────────────────────────────────────────────
void runInitYellow(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();

    robot.setSpeedPct(10); // Change la vitesse et accel pour le calage, afin de limiter les risques de rebonds

    // ── Calage X — plaquage contre la bordure Ouest ────────────────────────
    robot.setPosition(0, 0, ANGLE_EAST);
    robot.goStall(-150);                 // recule jusqu'au contact de la bordure Ouest
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_EAST);   // X calé
    robot.goPID(375-ROBOT_BACK_TO_CENTER_MM);                       // dégage

    // ── Calage Y — plaquage contre la bordure Nord ─────────────────────────
    robot.turnPID(-90.0f);               // s'oriente vers le Sud
    robot.goStall(-150);                 // recule jusqu'au contact de la bordure Nord
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);  // Y calé, X inchangé
    robot.goPID(225);                       // dégage

    // ── Position de départ ─────────────────────────────────────────────────
    robot.gotoXYenc(POI::startYellow,283);

    robot.resetSpeed(); // remet les vitesses par défaut pour la stratégie
}

// ─── Calage bordure bleu ──────────────────────────────────────────────────────
void runInitBlue(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();

    robot.setSpeedPct(10);

    // ── Calage X — plaquage contre la bordure Est ──────────────────────────
    robot.setPosition(TABLE_WIDTH_MM, 0, ANGLE_WEST);
    robot.goStall(-150);                  // recule (Est) contre la bordure Est
    robot.setPosition(TABLE_WIDTH_MM - ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_WEST);
    robot.goPID(TABLE_WIDTH_MM - ROBOT_BACK_TO_CENTER_MM - POI::startBlue.x);  // dégage vers l'Ouest

    // ── Calage Y — plaquage contre la bordure Nord ─────────────────────────
    robot.turnPID(90.0f);                 // s'oriente vers le Sud (180°→270°)
    robot.goStall(-150);                  // recule (Nord) contre la bordure Nord
    robot.setPosition(robot.getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH);
    robot.goPID(POI::startBlue.y);        // dégage vers le Sud

    // ── Position de départ ─────────────────────────────────────────────────
    robot.gotoXYenc(POI::startBlue, 257);

    robot.resetSpeed();
}

// ─── Stratégie jaune ──────────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot) {
    robot.enableObstacle();
    robot.setSpeedPct(80, 80);

    takeStock (robot, POI::stockYellow_01, ANGLE_WEST);

    robot.gotoXYenc(400, 1200);
    deposeStock(robot, 15, 1200, ANGLE_WEST);
    //robot.gotoXYenc(500, 1200);

    takeStock (robot, POI::stockYellow_02, ANGLE_WEST);

    robot.gotoXYenc(500, 1200);
    deposeStock(robot, 35, 1200, ANGLE_WEST);
    fastFermerGripper();
    robot.goPID(-150);
    //robot.gotoXYenc(300, 1200);

    // thermomètre : approche en ligne droite avec détection de blocage
    robot.setSpeedPct(80);
    robot.gotoXYenc(250,1850,ANGLE_EAST, REAR);
    //robot.goPID(-150);
    robot.setSpeedPct(40, 40);
    robot.goStall(-350);
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, robot.getY(), ANGLE_EAST);   // X calé
    deployerBrasDroit();
    robot.goPID(625);
    retracteBrasDroit();
    //robot.turnPID(90);
    robot.goPID(100);
    robot.setSpeedPct(80);

    //takeStock (robot, POI::stockYellow_04, ANGLE_NORTH);
    robot.gotoXYenc(975, 1200);
    robot.gotoXYenc(900,650, ANGLE_WEST);

    //robot.disableMotors();
}

// ─── Stratégie bleue — symétrique jaune (axe X = 1500mm) ─────────────────────
void runStrategyBlue(Robot &robot) {
    robot.enableObstacle();
    robot.setSpeedPct(80, 80);

    takeStock(robot, POI::stockBlue_01, ANGLE_EAST);

    robot.gotoXYenc(3000-400, 1200);
    deposeStock(robot, 2985, 1200, ANGLE_EAST);
    //robot.gotoXYenc(3000-500, 1200);

    takeStock(robot, POI::stockBlue_02, ANGLE_EAST);

    robot.gotoXYenc(2500, 1200);
    deposeStock(robot, 2965, 1200, ANGLE_EAST);
    fastFermerGripper();
    robot.goPID(-150);
    //robot.gotoXYenc(2700, 1200);

    // thermomètre : recalage contre bordure Est, sweep vers l'Ouest
    robot.setSpeedPct(80);
    robot.gotoXYenc(3000-250, 1850, ANGLE_WEST, REAR);
    //robot.goPID(-150);                    // recule (Est) vers la bordure Est
    robot.setSpeedPct(30, 30);
    robot.goStall(-350);            // plaque contre la bordure Est
    robot.setPosition(TABLE_WIDTH_MM - ROBOT_BACK_TO_CENTER_MM, robot.getY(), ANGLE_WEST);
    deployerBrasGauche();
    robot.goPID(625);                     // avance (Ouest) 625mm
    retracteBrasGauche();
    robot.goPID(100);
    robot.setSpeedPct(80);

    //takeStock(robot, POI::stockBlue_04, ANGLE_NORTH);
    robot.gotoXYenc(3000-975, 1200);
    robot.gotoXYenc(3000-900, 650, ANGLE_EAST);

    //robot.disableMotors();
}

// ─── Repli fin de match jaune ─────────────────────────────────────────────────
void runNearEndYellow(Robot &robot) {
    robot.enableMotors();
    robot.enableObstacle();

    robot.waitMatchTime(92000);
    robot.setSpeedPct(130);

    robot.gotoXYenc(400,650);
    robot.gotoXYenc(400,100);
    //deposeStock(robot,250,100,ANGLE_NORTH);
    libererStock();
    robot.disableMotors();
}

// ─── Repli fin de match bleu — symétrique jaune ───────────────────────────────
void runNearEndBlue(Robot &robot) {
    robot.enableMotors();
    robot.enableObstacle();
    
    robot.waitMatchTime(92000);
    robot.setSpeedPct(130);

    robot.gotoXYenc(3000-400,650);
    robot.gotoXYenc(3000-400,100);
    //deposeStock(robot, 2750, 100, ANGLE_NORTH);
    libererStock();
    robot.disableMotors();
}
