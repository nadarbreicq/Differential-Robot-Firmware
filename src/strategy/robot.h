#pragma once
#include "../motion/step_control.h"
#include "../lidar/ld06.h"
#include "../config.h"

// Interface haut niveau du robot.
// Toutes les méthodes sont BLOQUANTES : elles rendent la main quand le mouvement
// est terminé (ou interrompu par un obstacle).
class Robot {
public:
    Robot(StepControl &motion, LD06 &lidar);

    // ── Déplacements ─────────────────────────────────────────────────────────
    void go(float mm);                              // relatif, positif = avant
    void turn(float deg);                           // relatif, positif = gauche
    void gotoXY(float x_mm, float y_mm);            // absolu, angle d'arrivée libre
    void gotoXY(float x_mm, float y_mm, float arrival_deg); // absolu + angle final

    // ── Pose ─────────────────────────────────────────────────────────────────
    // Datum, recalage bordure, initialisation manuelle
    void setPosition(float x_mm, float y_mm, float theta_deg);

    // ── Cinématique ───────────────────────────────────────────────────────────
    void setSpeed(float mmS);

    // ── Détection obstacle ───────────────────────────────────────────────────
    void enableObstacle()             { _obstacleEn = true;  }
    void disableObstacle()            { _obstacleEn = false; }
    bool obstacleEnabled() const      { return _obstacleEn; }

    // ── Accesseurs état ──────────────────────────────────────────────────────
    float getX()        const { return _motion.getX(); }
    float getY()        const { return _motion.getY(); }
    float getTheta()    const { return _motion.getTheta(); }
    float getThetaDeg() const { return _motion.getThetaDeg(); }

private:
    StepControl &_motion;
    LD06        &_lidar;
    bool         _obstacleEn = true;

    LidarPoint   _scanBuf[LD06_SCAN_BUF_SIZE];

    // Retourne true si un obstacle est détecté dans le sens de déplacement
    // dir_rad : direction du mouvement en radians (repère monde)
    bool _obstacleInDir(float dir_rad);

    // Transforme un point LIDAR (angle en °, dist en mm) en coordonnées monde
    void _lidarToWorld(float angle_deg, float dist_mm,
                       float &wx, float &wy) const;

    // Bloque jusqu'à ce que l'obstacle disparaisse (ou timeout OBS_WAIT_MS)
    void _waitObstacleClear(float dir_rad);

    // Normalise un angle entre -π et π
    static float _normAngle(float rad);
};
