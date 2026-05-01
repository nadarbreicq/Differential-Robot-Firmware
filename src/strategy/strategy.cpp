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

// ─── Calage bordure jaune ─────────────────────────────────────────────────────
void runInitYellow(Robot &robot) {
    robot.disableObstacle();
    robot.go(-20);
    // TODO: plaquer contre les deux bordures côté jaune et enregistrer la pose réelle
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 900, 0);
}

// ─── Calage bordure bleu ──────────────────────────────────────────────────────
void runInitBlue(Robot &robot) {
    robot.disableObstacle();
    robot.go(-20);
    // TODO: plaquer contre les deux bordures côté bleu (symétrique jaune)
    robot.setPosition(TABLE_WIDTH_MM / 2, TABLE_HEIGHT_MM / 2, 0);
}

// ─── Test détection obstacle ──────────────────────────────────────────────────
// Le robot fait des allers-retours indéfiniment.
// Placer un objet devant ou derrière pour valider l'arrêt dans chaque sens.
void runTestObstacle(Robot &robot) {
    robot.enableObstacle();
    robot.setPosition(0, 0, 0);

    for (;;) {
        robot.go( 1500);                    // avance — détection avant
        vTaskDelay(pdMS_TO_TICKS(1000));
        robot.go(-1500);                    // recule — détection arrière
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ─── Stratégie jaune ──────────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot) {
    robot.enableObstacle();
    robot.setPosition(0, 0, 0);

    robot.go(500);
    robot.turn(-90);
    robot.turn(90);

    robot.gotoXY(1000, 500);
    robot.gotoXY(500, 500, 180);

    robot.disableMotors();
}

// ─── Stratégie bleue ─────────────────────────────────────────────────────────
void runStrategyBlue(Robot &robot) {
    robot.enableObstacle();
    robot.setPosition(0, 0, 0);

    robot.go(500);
    robot.turn(90);

    robot.gotoXY(1000, -500);
    robot.gotoXY(0, 0, 180);

    robot.disableMotors();
}
