#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "step_control.h"
#include "encoder.h"
#include "../lidar/ld06.h"
#include "../config.h"

// ─── MotionController ────────────────────────────────────────────────────────
//
//  Tâche FreeRTOS continue (50 Hz) qui pilote les moteurs vers une cible (X,Y,θ).
//
//  Machine d'états :
//    IDLE     → moteurs libres, tâche inactive
//    MOVING   → PID actif vers cible, sous-phases ALIGN → TRANSLATE → FINAL_TURN
//    HOLD     → PID maintien position (cible = pose au moment du holdPosition)
//    OBSTACLE → arrêté, attente dégagement avant reprise de la phase courante
//
//  Usage stratégie :
//    MotionController::Target t;
//    t.x_mm = 1000; t.y_mm = 500; t.theta_deg = 90;
//    motionCtrl.setTarget(t);
//    motionCtrl.waitArrived();   // bloquant
//
//  Thread safety : la cible est protégée par portMUX (écrite par taskStrategy
//  Core 1, lue par taskMotion Core 1). Les compteurs de séquence permettent
//  à waitArrived de détecter convergence ou préemption.
//
class MotionController {
public:
    enum class State : uint8_t {
        IDLE,       // moteurs libres, tâche inactive
        MOVING,     // PID actif vers cible
        HOLD,       // PID maintien position
        OBSTACLE,   // arrêt, attente dégagement
    };

    enum class Phase : uint8_t {
        NONE,
        ALIGN,          // rotation sur place vers la direction de la cible
        TRANSLATE,      // translation vers (x, y)
        FINAL_TURN,     // rotation finale vers theta
    };

    enum class DetectMode : uint8_t { SIMPLE, WALL_FILTERED };

    struct Target {
        float x_mm     = 0.0f;
        float y_mm     = 0.0f;
        float theta_deg = NAN;       // NaN = pas de contrainte d'orientation finale
        float speed    = 0.0f;       // mm/s ; 0 = utiliser gCalib.defaultSpeed
        float accel    = 0.0f;       // mm/s² ; 0 = utiliser gCalib.defaultAccel
        float stopMm   = 0.0f;       // mm ; 0 = utiliser gCalib.stopMm
        bool  backward = false;      // forcer approche en marche arrière
        bool  obstacleEn = true;     // activer détection pendant TRANSLATE
    };

    void begin(StepControl* motion, QuadEncoder* encL, QuadEncoder* encR, LD06* lidar);

    // ── API non-bloquante ────────────────────────────────────────────────────
    void setTarget(const Target& t);
    void holdPosition();
    void release();                  // → IDLE, moteurs libres

    // ── API bloquante ────────────────────────────────────────────────────────
    // Bloque jusqu'à convergence (ou distance < blendMm si > 0).
    // Retourne true si arrivé normalement, false si préempté par un nouveau
    // setTarget ou release().
    bool waitArrived(float blendMm = 0.0f);

    // ── État (lecture) ───────────────────────────────────────────────────────
    State    getState() const     { return _state; }
    Phase    getPhase() const     { return _phase; }
    float    getDistMm() const    { return _distMm; }
    float    getAngleDeg() const  { return _angleDeg; }
    uint32_t getUserSeq() const   { return _userSeq; }
    uint32_t getDoneSeq() const   { return _doneSeq; }
    // Debug : vitesses commandées (mm/s, signées) et cap courant
    float    getCmdL() const      { return _lastCmdL; }
    float    getCmdR() const      { return _lastCmdR; }
    float    getSpeedCap() const  { return _phaseSpeedCap; }
    // Cible courante (valide quand state != IDLE)
    float    getTargetX() const     { return _activeTgt.x_mm; }
    float    getTargetY() const     { return _activeTgt.y_mm; }
    float    getTargetTheta() const { return _activeTgt.theta_deg; }

    // ── Obstacle ─────────────────────────────────────────────────────────────
    void setDetectMode(DetectMode m) { _detectMode = m; }
    DetectMode getDetectMode() const { return _detectMode; }

private:
    static void _taskTrampoline(void* arg);
    void        _loop();

    // Transitions de phase
    void _initAlignPhase();
    void _initTranslatePhase();
    void _initFinalTurnPhase();
    void _initHold();
    void _completeTarget();
    // Blend : transition fluide vers une nouvelle cible pendant un TRANSLATE.
    // Préserve _phaseSpeedCap et compense la courbure via différentiel d'arc
    // entre roues. speedScale ∈ [0, 1] applique la règle de sécurité 30°/60°.
    void _initBlendPhase(float turnErr, float speedScale);
    // Vérifie si la transition vers _activeTgt est éligible au blend.
    // À appeler quand state == MOVING && phase == TRANSLATE.
    // Retourne true si _initBlendPhase a été appelé.
    bool _tryBlendTransition();

    // PID
    bool _runPidStep();           // dispatcher selon la phase courante
    bool _runWheelPidStep();      // wheel-PID classique (ALIGN, FINAL_TURN)
    bool _runPoseStep();          // contrôleur pose-based (TRANSLATE)
    void _resetPidState();
    bool _checkObstacle();       // true si obstacle détecté dans direction courante

    // Détection obstacle (porté de Robot)
    bool _obstacleInDir(float dir_rad);
    bool _obstacleSimple(float dir_rad);
    bool _obstacleWallFiltered(float dir_rad);
    void _lidarToWorld(float angle_deg, float dist_mm, float& wx, float& wy) const;

    static float _normAngle(float rad);

    StepControl* _motion = nullptr;
    QuadEncoder* _encL = nullptr;
    QuadEncoder* _encR = nullptr;
    LD06*        _lidar = nullptr;

    // Cible utilisateur (partagée, protégée par mux)
    Target            _userTgt;
    volatile uint32_t _userSeq = 0;
    portMUX_TYPE      _mux = portMUX_INITIALIZER_UNLOCKED;

    // État interne (task only)
    volatile State    _state = State::IDLE;
    volatile Phase    _phase = Phase::NONE;
    uint32_t          _handledSeq = 0;
    volatile uint32_t _doneSeq = 0;

    // État PID de la phase courante
    Target  _activeTgt;              // copie locale travaillée
    bool    _activeGoBack = false;
    int32_t _phaseTL = 0, _phaseTR = 0;
    float   _phaseSpeed = 0, _phaseAccel = 0, _phaseStopMm = 0;
    float   _phaseIL = 0, _phaseIR = 0;
    float   _phasePrevEL = 0, _phasePrevER = 0;
    float   _phaseSpeedCap = 0;
    float   _phaseMoveDir = 0;
    bool    _phaseObstacleEn = false;

    // OBSTACLE state
    uint32_t _obstacleStartMs = 0;
    bool     _obstacleBackedUp = false;

    // Flag : true = phase TRANSLATE utilise le contrôleur pose-based
    // (set par _initBlendPhase, reset par _initTranslatePhase classique).
    // Permet d'avoir le wheel-PID inchangé pour les gotoXYenc/goPID standards.
    bool _useBlendController = false;

    // Affichage debug
    volatile float _distMm = 0;
    volatile float _angleDeg = 0;
    volatile float _lastCmdL = 0;     // dernière commande envoyée à setMotorVelocities
    volatile float _lastCmdR = 0;

    // Obstacle
    LidarPoint  _scanBuf[LD06_SCAN_BUF_SIZE];
    DetectMode  _detectMode = DetectMode::SIMPLE;
};

extern MotionController gMotionCtrl;
