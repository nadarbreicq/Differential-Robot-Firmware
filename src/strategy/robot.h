#pragma once
#include "../motion/step_control.h"
#include "../motion/encoder.h"
#include "../lidar/ld06.h"
#include "../config.h"
#include "poi.h"

// Interface haut niveau du robot.
// Toutes les méthodes sont BLOQUANTES : elles rendent la main quand le mouvement
// est terminé (ou interrompu par un obstacle).
class Robot {
public:
    Robot(StepControl &motion, LD06 &lidar);

    // ── Déplacements ─────────────────────────────────────────────────────────
    void go(float mm);                              // relatif, step-based
    bool goStall(float mm, uint32_t timeoutMs = 3000); // go avec détection blocage encodeurs
    void goPID(float mm);                           // relatif, asservi encodeurs
    void turn(float deg);                           // relatif, positif = gauche
    void gotoXY(float x_mm, float y_mm);
    void gotoXY(float x_mm, float y_mm, float arrival_deg);
    void gotoXY(Vec2 poi)                    { gotoXY(poi.x, poi.y); }
    void gotoXY(Vec2 poi, float arrival_deg) { gotoXY(poi.x, poi.y, arrival_deg); }

    // Navigation odométrie encodeurs — double PID position
    void turnPID(float deg);
    void gotoXYenc(float x_mm, float y_mm);
    void gotoXYenc(float x_mm, float y_mm, float arrival_deg);
    void gotoXYenc(Vec2 poi)                    { gotoXYenc(poi.x, poi.y); }
    void gotoXYenc(Vec2 poi, float arrival_deg) { gotoXYenc(poi.x, poi.y, arrival_deg); }
    void setEncoders(QuadEncoder *l, QuadEncoder *r) { _encLeft = l; _encRight = r; }

    // ── Pose ─────────────────────────────────────────────────────────────────
    void setPosition(float x_mm, float y_mm, float theta_deg);

    // ── Cinématique ───────────────────────────────────────────────────────────
    void setSpeed(float mmS);
    void setSpeedPct(float speedPct, float accelPct = -1.0f);  // % de DEFAULT_SPEED/ACCEL (-1 = même % que speed)
    void resetSpeed();                                          // revient aux valeurs par défaut

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
    void     startNearEnd() { _nearEndMode = true; }  // désactive les interruptions isEndgame()
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

    bool          _nearEndMode = false;
    LidarPoint    _scanBuf[LD06_SCAN_BUF_SIZE];
    QuadEncoder  *_encLeft  = nullptr;
    QuadEncoder  *_encRight = nullptr;

    void _runWheelPID(int32_t tL, int32_t tR, float speed, float accel, float stopMm = ENC_P1_STOP_MM);

    bool _obstacleInDir(float dir_rad);
    bool _obstacleSimple(float dir_rad);
    bool _obstacleWallFiltered(float dir_rad);

    void _lidarToWorld(float angle_deg, float dist_mm,
                       float &wx, float &wy) const;

    void _waitObstacleClear(float dir_rad);

    static float _normAngle(float rad);
};
