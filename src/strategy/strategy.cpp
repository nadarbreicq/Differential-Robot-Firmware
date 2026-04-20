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
//  Repère : X+ = avant initial, Y+ = gauche, angle 0° = avant
//  Table  : 2000 mm × 3000 mm
// ═══════════════════════════════════════════════════════════════════════════════

#include "strategy.h"

void runStrategy(Robot &robot) {
    robot.disableObstacle(); 

    // ── Recalage initial ─────────────────────────────────────────────────────
    robot.setPosition(0, 0, 0);         // robot en (0,0), orienté vers X+

    // ── Exemple de séquence ──────────────────────────────────────────────────
    robot.go(500);                      // avance 500 mm
    robot.turn(-90);                    // tourne à droite 90°
    robot.go(300);                      // avance 300 mm

    //robot.disableObstacle();            // passage en zone connue sans adversaire
    robot.go(200);
    //robot.enableObstacle();

    robot.gotoXY(1000, 500);            // va en absolu (sans contrainte d'angle)
    robot.gotoXY(0, 0, 180);           // retour origine, orienté à 180°

}
