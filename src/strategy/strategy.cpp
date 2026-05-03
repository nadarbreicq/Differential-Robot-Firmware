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

// ─── Calage bordure jaune ─────────────────────────────────────────────────────
void runInitYellow(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    robot.go(-20);
    // TODO: plaquer contre les deux bordures côté jaune et enregistrer la pose réelle
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 900, 0);
}

// ─── Calage bordure bleu ──────────────────────────────────────────────────────
void runInitBlue(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    robot.go(-20);
    // TODO: plaquer contre les deux bordures côté bleu (symétrique jaune)
    robot.setPosition(TABLE_WIDTH_MM / 2, TABLE_HEIGHT_MM / 2, 0);
}

// ─── Stratégie jaune ──────────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot) {
    robot.enableObstacle();

    robot.go(500);
    deployerBras();              // déploie le bras à 150° à 60°/s

    robot.go(300);
    retracteBras();              // rétracte à 10° à 60°/s

    robot.gotoXY(1000, 500);
    servoBras.moveTo(90, 30);   // ou directement : 90° à 30°/s

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
