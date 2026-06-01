#pragma once
#include "../motion/step_control.h"
#include "../motion/motion_ctrl.h"
#include "../motion/encoder.h"
#include "../lidar/ld06.h"
#include "../config.h"
#include "poi.h"

// Direction de déplacement pour gotoXYenc
constexpr bool FORWARD = false;
constexpr bool REAR    = true;

// Interface haut niveau du robot.
// Toutes les méthodes sont BLOQUANTES : elles rendent la main quand le mouvement
// est terminé (ou interrompu par un obstacle).
class Robot {
public:
    Robot(StepControl &motion, LD06 &lidar, MotionController &motionCtrl);

    // ── Déplacements ─────────────────────────────────────────────────────────
    void go(float mm);                              // relatif, step-based
    // go avec détection blocage encodeurs.
    // timeoutMs   : durée max totale avant abandon (sécurité)
    // stallConfirmMs : durée min de non-mouvement pour valider le stall (anti-frottement)
    bool goStall(float mm, uint32_t timeoutMs = STALL_TIMEOUT_MS, uint32_t stallConfirmMs = STALL_CONFIRM_MS);
    void goPID(float mm);                           // relatif, asservi encodeurs
    void turn(float deg);                           // relatif, positif = gauche
    void gotoXY(float x_mm, float y_mm);
    void gotoXY(float x_mm, float y_mm, float arrival_deg);
    void gotoXY(Vec2 poi)                    { gotoXY(poi.x, poi.y); }
    void gotoXY(Vec2 poi, float arrival_deg) { gotoXY(poi.x, poi.y, arrival_deg); }

    // Navigation odométrie encodeurs — double PID position
    void turnPID(float deg);
    void gotoXYenc(float x_mm, float y_mm);
    void gotoXYenc(float x_mm, float y_mm, float arrival_deg, bool backward = false);
    void gotoXYenc(Vec2 poi)                                        { gotoXYenc(poi.x, poi.y); }
    void gotoXYenc(Vec2 poi, float arrival_deg, bool backward = false) { gotoXYenc(poi.x, poi.y, arrival_deg, backward); }
    void setEncoders(QuadEncoder *l, QuadEncoder *r) { _encLeft = l; _encRight = r; }

    // ── Pose ─────────────────────────────────────────────────────────────────
    void setPosition(float x_mm, float y_mm, float theta_deg);

    // ── Cinématique ───────────────────────────────────────────────────────────
    void  setSpeed(float mmS);
    void  setAcceleration(float mmS2)   { _motion.setAcceleration(mmS2); }
    void  setSpeedPct(float speedPct, float accelPct = -1.0f);
    void  resetSpeed();
    float getSpeed()        const { return _motion.getSpeed(); }
    float getAcceleration() const { return _motion.getAcceleration(); }

    // ── API motion non-bloquante (taskMotionControl) ─────────────────────────
    // Met à jour la cible courante de la tâche motion (retour immédiat).
    void setTarget(float x_mm, float y_mm);
    void setTarget(float x_mm, float y_mm, float arrival_deg, bool backward = false);
    void setTarget(Vec2 poi)                                   { setTarget(poi.x, poi.y); }
    void setTarget(Vec2 poi, float arrival_deg, bool backward = false) {
        setTarget(poi.x, poi.y, arrival_deg, backward);
    }
    // Maintient la pose courante en mode HOLD.
    void holdPosition() { _motionCtrl.holdPosition(); }
    // Libère la tâche motion → IDLE (moteurs en roue libre).
    void releaseMotion() { _motionCtrl.release(); }
    // Bloque jusqu'à convergence (ou distance < blendMm si > 0).
    // Surveille aussi match over / endgame et libère la tâche motion.
    bool waitArrived(float blendMm = 0.0f);
    MotionController::State getMotionState() const { return _motionCtrl.getState(); }

    // ── Détection obstacle ───────────────────────────────────────────────────
    enum class DetectMode : uint8_t {
        SIMPLE,
        WALL_FILTERED,
    };
    void enableObstacle()             { _obstacleEn = true;  }
    void disableObstacle()            { _obstacleEn = false; }
    bool obstacleEnabled() const      { return _obstacleEn; }
    void setDetectMode(DetectMode m)  {
        _detectMode = m;
        _motionCtrl.setDetectMode(m == DetectMode::WALL_FILTERED
            ? MotionController::DetectMode::WALL_FILTERED
            : MotionController::DetectMode::SIMPLE);
    }

    // ── Moteurs ──────────────────────────────────────────────────────────────
    void disableMotors()  { _motionCtrl.release(); _motion.disableMotors(); }
    void enableMotors()   { _motion.enableMotors();  }

    // ── Chrono de match ──────────────────────────────────────────────────────
    void     startMatch();
    void     startNearEnd() { _nearEndMode = true; }  // désactive les interruptions isEndgame()
    uint32_t matchElapsed() const;
    bool     isEndgame()    const;
    bool     isMatchOver()  const;
    // Bloque jusqu'à ce que le chrono de match atteigne target_ms (ou fin de match).
    // Usage dans runNearEndXxx : robot.waitMatchTime(95000); // attend la 95e seconde
    void     waitMatchTime(uint32_t target_ms);

    // ── Accesseurs état ──────────────────────────────────────────────────────
    float getX()        const { return _motion.getX(); }
    float getY()        const { return _motion.getY(); }
    float getTheta()    const { return _motion.getTheta(); }
    float getThetaDeg() const { return _motion.getThetaDeg(); }

private:
    StepControl       &_motion;
    LD06              &_lidar;
    MotionController  &_motionCtrl;
    bool               _obstacleEn = true;
    DetectMode         _detectMode = DetectMode::SIMPLE;

    bool          _nearEndMode = false;
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
