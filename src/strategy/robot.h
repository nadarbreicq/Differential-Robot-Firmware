#pragma once
#include "../motion/step_control.h"
#include "../motion/encoder.h"
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
    void gotoXY(float x_mm, float y_mm);            // dead reckoning (pas commandés)
    void gotoXY(float x_mm, float y_mm, float arrival_deg);

    // Navigation odométrie encodeurs — Principe 1 RCVA
    void turnEnc(float deg);                               // rotation PID par roue
    void gotoXYenc(float x_mm, float y_mm);
    void gotoXYenc(float x_mm, float y_mm, float arrival_deg);
    void setEncoders(QuadEncoder *l, QuadEncoder *r) { _encLeft = l; _encRight = r; }

    // ── Pose ─────────────────────────────────────────────────────────────────
    void setPosition(float x_mm, float y_mm, float theta_deg);

    // ── Cinématique ───────────────────────────────────────────────────────────
    void setSpeed(float mmS);

    // ── Détection obstacle ───────────────────────────────────────────────────
    enum class DetectMode : uint8_t {
        SIMPLE,
        WALL_FILTERED,
    };
    void enableObstacle()             { _obstacleEn = true;  }
    void disableObstacle()            { _obstacleEn = false; }
    bool obstacleEnabled() const      { return _obstacleEn; }
    void setDetectMode(DetectMode m)  { _detectMode = m; }

    // ── Moteurs ──────────────────────────────────────────────────────────────
    void disableMotors()  { _motion.disableMotors(); }
    void enableMotors()   { _motion.enableMotors();  }

    // ── Chrono de match ──────────────────────────────────────────────────────
    void     startMatch();
    uint32_t matchElapsed() const;
    bool     isEndgame()    const;
    bool     isMatchOver()  const;

    // ── Accesseurs état ──────────────────────────────────────────────────────
    float getX()        const { return _motion.getX(); }
    float getY()        const { return _motion.getY(); }
    float getTheta()    const { return _motion.getTheta(); }
    float getThetaDeg() const { return _motion.getThetaDeg(); }

private:
    StepControl &_motion;
    LD06        &_lidar;
    bool         _obstacleEn = true;
    DetectMode   _detectMode = DetectMode::SIMPLE;

    LidarPoint    _scanBuf[LD06_SCAN_BUF_SIZE];
    QuadEncoder  *_encLeft  = nullptr;
    QuadEncoder  *_encRight = nullptr;

    bool _obstacleInDir(float dir_rad);
    bool _obstacleSimple(float dir_rad);
    bool _obstacleWallFiltered(float dir_rad);

    void _lidarToWorld(float angle_deg, float dist_mm,
                       float &wx, float &wy) const;

    void _waitObstacleClear(float dir_rad);

    static float _normAngle(float rad);
};
