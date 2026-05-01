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
    enum class DetectMode : uint8_t {
        SIMPLE,        // distance brute dans un rectangle — pas de filtre murs
        WALL_FILTERED, // filtre les points identifiés comme murs (nécessite pose réelle)
    };

    void enableObstacle()             { _obstacleEn = true;  }
    void disableObstacle()            { _obstacleEn = false; }
    bool obstacleEnabled() const      { return _obstacleEn; }
    void setDetectMode(DetectMode m)  { _detectMode = m; }

    // ── Moteurs ──────────────────────────────────────────────────────────────
    void disableMotors()  { _motion.disableMotors(); }
    void enableMotors()   { _motion.enableMotors();  }

    // ── Accesseurs état ──────────────────────────────────────────────────────
    float getX()        const { return _motion.getX(); }
    float getY()        const { return _motion.getY(); }
    float getTheta()    const { return _motion.getTheta(); }
    float getThetaDeg() const { return _motion.getThetaDeg(); }

private:
    StepControl &_motion;
    LD06        &_lidar;
    bool         _obstacleEn  = true;
    DetectMode   _detectMode  = DetectMode::SIMPLE;

    LidarPoint   _scanBuf[LD06_SCAN_BUF_SIZE];

    bool _obstacleInDir(float dir_rad);
    bool _obstacleSimple(float dir_rad);       // sans filtre murs
    bool _obstacleWallFiltered(float dir_rad); // avec filtre murs (pose réelle requise)

    void _lidarToWorld(float angle_deg, float dist_mm,
                       float &wx, float &wy) const;

    // Bloque jusqu'à ce que l'obstacle disparaisse (ou timeout OBS_WAIT_MS)
    void _waitObstacleClear(float dir_rad);

    // Normalise un angle entre -π et π
    static float _normAngle(float rad);
};
