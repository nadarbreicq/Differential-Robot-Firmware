#include "motion_ctrl.h"
#include "../live_config.h"
#include "../display/oled.h"
#include "../log.h"
#include <math.h>

MotionController gMotionCtrl;

static constexpr float PI_F    = 3.14159265f;
static constexpr float DEG2RAD = PI_F / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI_F;

// ─── begin ───────────────────────────────────────────────────────────────────

void MotionController::begin(StepControl* motion, QuadEncoder* encL,
                              QuadEncoder* encR, LD06* lidar) {
    _motion = motion;
    _encL   = encL;
    _encR   = encR;
    _lidar  = lidar;

    xTaskCreatePinnedToCore(_taskTrampoline, "motion", 4096, this, 3, nullptr, 1);
}

void MotionController::_taskTrampoline(void* arg) {
    static_cast<MotionController*>(arg)->_loop();
}

// ─── API non-bloquante ───────────────────────────────────────────────────────

void MotionController::setTarget(const Target& t) {
    portENTER_CRITICAL(&_mux);
    _userTgt = t;
    if (_userTgt.speed   <= 0.0f) _userTgt.speed   = gCalib.defaultSpeed;
    if (_userTgt.accel   <= 0.0f) _userTgt.accel   = gCalib.defaultAccel;
    if (_userTgt.stopMm  <= 0.0f) _userTgt.stopMm  = gCalib.stopMm;
    _userSeq++;
    portEXIT_CRITICAL(&_mux);
    // Empêche waitArrived(blendMm) de sortir prématurément avant que la tâche
    // motion ait pris en compte la nouvelle cible (1 cycle = 20 ms).
    // Sans ça, _distMm garde sa valeur du mouvement précédent (souvent 0).
    _distMm = 1e9f;
}

void MotionController::holdPosition() {
    portENTER_CRITICAL(&_mux);
    // Cible spéciale : signale HOLD via theta_deg = -9999 sentinel
    // (HOLD pur géré dans la tâche en lisant la pose actuelle)
    _userTgt = Target{};
    _userTgt.x_mm    = gDisplay.enc_pose_x_mm;
    _userTgt.y_mm    = gDisplay.enc_pose_y_mm;
    _userTgt.theta_deg = gDisplay.enc_pose_theta_deg;
    _userTgt.speed   = gCalib.defaultSpeed;
    _userTgt.accel   = gCalib.defaultAccel;
    _userTgt.stopMm  = gCalib.stopMm;
    _userTgt.backward = false;
    _userTgt.obstacleEn = false;
    _userSeq++;
    portEXIT_CRITICAL(&_mux);
    // Note : le HOLD complet (résistance active) sera implémenté en 1.3.
    // Pour l'instant, holdPosition() = setTarget(pose courante) + l'inertie du
    // PID corrige naturellement les petits déplacements imposés.
}

void MotionController::release() {
    portENTER_CRITICAL(&_mux);
    _state = State::IDLE;
    _phase = Phase::NONE;
    _doneSeq = _userSeq;   // déverrouille tout waitArrived en cours
    portEXIT_CRITICAL(&_mux);
    if (_motion) _motion->softStop();
    _lastCmdL = 0; _lastCmdR = 0;
}

bool MotionController::waitArrived(float blendMm) {
    portENTER_CRITICAL(&_mux);
    uint32_t expected = _userSeq;
    portEXIT_CRITICAL(&_mux);

    while (true) {
        portENTER_CRITICAL(&_mux);
        uint32_t curUser = _userSeq;
        uint32_t curDone = _doneSeq;
        portEXIT_CRITICAL(&_mux);

        if (curUser != expected) return false;       // préempté par nouveau setTarget
        if (curDone == expected) return true;        // arrivé / terminé / release
        if (blendMm > 0.0f && _distMm < blendMm) return true;
        // PAS de check State::IDLE : la tâche motion (50 Hz) peut ne pas avoir
        // encore vu le nouveau seq → seq est la seule source de vérité.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Boucle principale (tâche) ───────────────────────────────────────────────

void MotionController::_loop() {
    while (true) {
        // Lire seq utilisateur
        portENTER_CRITICAL(&_mux);
        uint32_t seq = _userSeq;
        Target   tgtCopy = _userTgt;
        portEXIT_CRITICAL(&_mux);

        // Nouvelle cible ?
        if (seq != _handledSeq) {
            _handledSeq = seq;
            _activeTgt  = tgtCopy;

            // Tente un blend si on était déjà en TRANSLATE — sinon init normal
            bool blended = false;
            if (_state == State::MOVING && _phase == Phase::TRANSLATE) {
                blended = _tryBlendTransition();
            }
            if (!blended) {
                _state = State::MOVING;
                _initAlignPhase();
            }
        }

        switch (_state) {
            case State::IDLE:
                _distMm = 0;
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;

            case State::MOVING: {
                bool phaseDone = _runPidStep();

                // Détection obstacle (uniquement quand activée pour cette phase)
                if (_phaseObstacleEn && _obstacleInDir(_phaseMoveDir)) {
                    _motion->softStop(OBS_STOP_ACCEL_MMS2);
                    LOG_W("MOTION", "Obstacle %.0fmm — recul + attente",
                          (double)gDisplay.obs_dist_mm);
                    gDisplay.robot_state = RobotState::OBSTACLE;
                    _state = State::OBSTACLE;
                    _obstacleStartMs = millis();
                    _obstacleBackedUp = false;
                    break;
                }

                if (phaseDone) {
                    switch (_phase) {
                        case Phase::ALIGN:      _initTranslatePhase(); break;
                        case Phase::TRANSLATE:
                            // En blend : on arrive "proche", pas d'ajustement final.
                            // Le path-following compte sur la vitesse, pas la précision.
                            if (_useBlendController) _completeTarget();
                            else                     _initFinalTurnPhase();
                            break;
                        case Phase::FINAL_TURN: _completeTarget();     break;
                        default:                _completeTarget();     break;
                    }
                }
                break;
            }

            case State::HOLD:
                // Pour l'instant, HOLD = MOVING avec cible figée (cf. 1.3 pour évolution)
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;

            case State::OBSTACLE: {
                // Recul une fois (100 mm), puis attente dégagement avec timeout
                if (!_obstacleBackedUp) {
                    // Recul léger (utilise StepControl direct, ne change pas la phase)
                    _motion->go(_activeGoBack ? OBS_BACKUP_MM : -OBS_BACKUP_MM);
                    _obstacleBackedUp = true;
                    _obstacleStartMs  = millis();
                    // Re-init la phase TRANSLATE depuis la nouvelle position
                    _initTranslatePhase();
                    break;
                }

                bool clear = !_obstacleInDir(_phaseMoveDir);
                bool timeout = (millis() - _obstacleStartMs) > OBS_WAIT_MS;
                if (clear || timeout) {
                    if (timeout) LOG_W("MOTION", "Timeout obstacle — reprise");
                    else         LOG_I("MOTION", "Obstacle dégagé — reprise");
                    _resetPidState();
                    _phaseSpeedCap = gCalib.minSpd;
                    _state = State::MOVING;
                    gDisplay.robot_state = RobotState::GOTO;
                }
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));
    }
}

// ─── Phases ──────────────────────────────────────────────────────────────────

void MotionController::_initAlignPhase() {
    const float curX     = gDisplay.enc_pose_x_mm;
    const float curY     = gDisplay.enc_pose_y_mm;
    const float curTheta = gDisplay.enc_pose_theta_rad;

    const float dx = _activeTgt.x_mm - curX;
    const float dy = _activeTgt.y_mm - curY;
    const float dist = sqrtf(dx*dx + dy*dy);

    // Déjà au point ? → skip TRANSLATE, passer au FINAL_TURN éventuel
    if (dist < 1.0f) {
        _activeGoBack = _activeTgt.backward;
        _initFinalTurnPhase();
        return;
    }

    float aerr = _normAngle(atan2f(-dy, dx) - curTheta);
    _activeGoBack = _activeTgt.backward || fabsf(aerr) > PI_F * 0.5f;
    float turnErr = _activeGoBack
                    ? _normAngle(aerr + (aerr >= 0 ? -PI_F : PI_F))
                    : aerr;

    // Déjà aligné (< 3°) ? → skip ALIGN
    if (fabsf(turnErr) < 0.05f) {
        _initTranslatePhase();
        return;
    }

    // Setup PID rotation sur place
    float arc  = gCalib.encWheelbase * fabsf(turnErr) * 0.5f;
    float sign = turnErr > 0 ? 1.0f : -1.0f;
    _phaseTL = _encL->getCount() - (int32_t)(sign * arc / gCalib.mmPerCount);
    _phaseTR = _encR->getCount() + (int32_t)(sign * arc / gCalib.mmPerCount);
    _phaseSpeed   = fminf(_activeTgt.speed, TURN_SPEED_MMS);
    _phaseAccel   = fminf(_activeTgt.accel, TURN_ACCEL_MMS2);
    _phaseStopMm  = ENC_P1_STOP_DEG * gCalib.encWheelbase * PI_F / 360.0f;
    _phaseMoveDir = curTheta;
    _phaseObstacleEn = false;   // pas de détection pendant rotation
    _resetPidState();
    _phase = Phase::ALIGN;
    _motion->setAcceleration(_phaseAccel);
    _motion->pushAcceleration();
    gDisplay.robot_state = RobotState::TURNING;
}

void MotionController::_initTranslatePhase() {
    _useBlendController = false;   // wheel-PID classique

    const float curX = gDisplay.enc_pose_x_mm;
    const float curY = gDisplay.enc_pose_y_mm;
    const float curTheta = gDisplay.enc_pose_theta_rad;

    const float dx = _activeTgt.x_mm - curX;
    const float dy = _activeTgt.y_mm - curY;
    const float dist = sqrtf(dx*dx + dy*dy);

    // Déjà au point ? → FINAL_TURN ou done
    if (dist < 1.0f) {
        _initFinalTurnPhase();
        return;
    }

    int32_t counts = (int32_t)(dist / gCalib.mmPerCount);
    int32_t sign   = _activeGoBack ? -1 : 1;
    _phaseTL = _encL->getCount() + sign * counts;
    _phaseTR = _encR->getCount() + sign * counts;
    _phaseSpeed   = _activeTgt.speed;
    _phaseAccel   = _activeTgt.accel;
    _phaseStopMm  = _activeTgt.stopMm;
    _phaseMoveDir = curTheta + (_activeGoBack ? PI_F : 0.0f);
    _phaseObstacleEn = _activeTgt.obstacleEn;
    _resetPidState();
    _phase = Phase::TRANSLATE;
    _motion->setAcceleration(_phaseAccel);
    _motion->pushAcceleration();
    gDisplay.robot_state = RobotState::MOVING;
}

void MotionController::_initFinalTurnPhase() {
    if (isnanf(_activeTgt.theta_deg)) {
        _completeTarget();
        return;
    }

    const float curThetaDeg = gDisplay.enc_pose_theta_deg;
    float deltaDeg = _activeTgt.theta_deg - curThetaDeg;
    while (deltaDeg >  180.0f) deltaDeg -= 360.0f;
    while (deltaDeg < -180.0f) deltaDeg += 360.0f;

    if (fabsf(deltaDeg) < 0.5f) {
        _completeTarget();
        return;
    }

    float deltaRad = deltaDeg * DEG2RAD;
    float arc  = gCalib.encWheelbase * fabsf(deltaRad) * 0.5f;
    float sign = deltaRad > 0 ? 1.0f : -1.0f;
    _phaseTL = _encL->getCount() - (int32_t)(sign * arc / gCalib.mmPerCount);
    _phaseTR = _encR->getCount() + (int32_t)(sign * arc / gCalib.mmPerCount);
    _phaseSpeed   = fminf(_activeTgt.speed, TURN_SPEED_MMS);
    _phaseAccel   = fminf(_activeTgt.accel, TURN_ACCEL_MMS2);
    _phaseStopMm  = ENC_P1_STOP_DEG * gCalib.encWheelbase * PI_F / 360.0f;
    _phaseMoveDir = gDisplay.enc_pose_theta_rad;
    _phaseObstacleEn = false;
    _resetPidState();
    _phase = Phase::FINAL_TURN;
    _motion->setAcceleration(_phaseAccel);
    _motion->pushAcceleration();
    gDisplay.robot_state = RobotState::TURNING;
}

void MotionController::_completeTarget() {
    _motion->softStop();
    // Synchronise pose dead-reckoning sur pose encodeurs (cohérence avec _runWheelPID legacy)
    vTaskDelay(pdMS_TO_TICKS(10));
    _motion->setPosition(gDisplay.enc_pose_x_mm,
                          gDisplay.enc_pose_y_mm,
                          gDisplay.enc_pose_theta_deg);

    _phase = Phase::NONE;
    _state = State::IDLE;
    _distMm = 0;
    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;

    portENTER_CRITICAL(&_mux);
    _doneSeq = _handledSeq;
    portEXIT_CRITICAL(&_mux);
}

// ─── Blend (transition fluide pendant TRANSLATE) ─────────────────────────────
//
//  Quand un nouveau setTarget arrive alors qu'on est en MOVING/TRANSLATE :
//  • angle < 30°       → blend plein (speedScale = 1)
//  • angle 30°-60°     → blend réduit (speedScale = cos(angle))
//  • angle ≥ 60°       → pas de blend (force stop + ALIGN classique)
//
//  Le blend conserve _phaseSpeedCap (pas de ramp depuis minSpd).
//  Une courbure approximative est appliquée via différentiel d'arc entre les
//  roues : avgCenter = sign·dist, halfDelta = wheelbase·turnErr/2.
//
bool MotionController::_tryBlendTransition() {
    const float curX     = gDisplay.enc_pose_x_mm;
    const float curY     = gDisplay.enc_pose_y_mm;
    const float curTheta = gDisplay.enc_pose_theta_rad;

    const float dx = _activeTgt.x_mm - curX;
    const float dy = _activeTgt.y_mm - curY;
    const float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1.0f) return false;

    const float aerr = _normAngle(atan2f(-dy, dx) - curTheta);
    const bool  goBack = _activeTgt.backward || fabsf(aerr) > PI_F * 0.5f;
    const float turnErr = goBack ? _normAngle(aerr + (aerr >= 0 ? -PI_F : PI_F)) : aerr;
    _activeGoBack = goBack;

    // Pose-based : le contrôleur gère tous les angles naturellement (v_lin∝cos(err)).
    // Plus de seuil 30°/60° — la transition est continue à n'importe quel angle.
    LOG_I("MOTION", "Blend %.0fmm angle=%.0fdeg", (double)dist, (double)(turnErr * RAD2DEG));
    _initBlendPhase(turnErr, 1.0f);
    return true;
}

// Init blend : reprend TRANSLATE sans reset du speedCap.
// Avec le contrôleur pose-based, plus besoin de cibles wheel ni d'arc compensé.
void MotionController::_initBlendPhase(float /*turnErr*/, float speedScale) {
    _useBlendController = true;   // dispatch vers _runPoseStep

    const float curX = gDisplay.enc_pose_x_mm;
    const float curY = gDisplay.enc_pose_y_mm;

    const float dx = _activeTgt.x_mm - curX;
    const float dy = _activeTgt.y_mm - curY;
    const float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1.0f) { _initFinalTurnPhase(); return; }

    // CLÉ DU BLEND : on conserve _phaseSpeedCap au lieu de le reset à minSpd
    const float savedSpeedCap = _phaseSpeedCap;

    _phaseSpeed   = _activeTgt.speed;
    _phaseAccel   = _activeTgt.accel;
    _phaseStopMm  = _activeTgt.stopMm;
    _phaseMoveDir = gDisplay.enc_pose_theta_rad + (_activeGoBack ? PI_F : 0.0f);
    _phaseObstacleEn = _activeTgt.obstacleEn;

    const float blendCap = savedSpeedCap * speedScale;
    _phaseSpeedCap = fmaxf(gCalib.minSpd, fminf(blendCap, _phaseSpeed));

    _phase = Phase::TRANSLATE;
    _motion->setAcceleration(_phaseAccel);
    _motion->pushAcceleration();
    gDisplay.robot_state = RobotState::MOVING;
}

// ─── PID step (une itération wheel-PID) ──────────────────────────────────────

void MotionController::_resetPidState() {
    _phaseIL = 0;
    _phaseIR = 0;
    _phasePrevEL = (float)(_phaseTL - _encL->getCount()) * gCalib.mmPerCount;
    _phasePrevER = (float)(_phaseTR - _encR->getCount()) * gCalib.mmPerCount;
    _phaseSpeedCap = gCalib.minSpd;
}

// Dispatcher : pose-based UNIQUEMENT pendant un blend (TRANSLATE entré via
// _initBlendPhase). Sinon wheel-PID classique (comportement original conservé
// pour gotoXYenc, goPID, turnPID, ALIGN, FINAL_TURN).
bool MotionController::_runPidStep() {
    if (_phase == Phase::TRANSLATE && _useBlendController) return _runPoseStep();
    return _runWheelPidStep();
}

// Wheel-PID classique : utilisé pour ALIGN et FINAL_TURN (rotations sur place
// où les roues ont des cibles count opposées).
bool MotionController::_runWheelPidStep() {
    const float dt = OBS_POLL_MS / 1000.0f;

    _phaseSpeedCap = fminf(_phaseSpeedCap + _phaseAccel * dt, _phaseSpeed);

    float eL = (float)(_phaseTL - _encL->getCount()) * gCalib.mmPerCount;
    float eR = (float)(_phaseTR - _encR->getCount()) * gCalib.mmPerCount;
    float avg = (fabsf(eL) + fabsf(eR)) * 0.5f;

    // _distMm = distance GÉOMÉTRIQUE à la cible XY (pour waitArrived(blendMm)).
    const float dgx = _activeTgt.x_mm - gDisplay.enc_pose_x_mm;
    const float dgy = _activeTgt.y_mm - gDisplay.enc_pose_y_mm;
    _distMm = sqrtf(dgx*dgx + dgy*dgy);

    gDisplay.nav_dist_mm   = avg;
    gDisplay.nav_delta_deg = eL - eR;

    if (avg <= _phaseStopMm) {
        _motion->softStop();
        _lastCmdL = 0; _lastCmdR = 0;
        return true;
    }

    // PID roue gauche
    _phaseIL = fmaxf(-gCalib.iMax, fminf(_phaseIL + gCalib.ki * eL * dt, gCalib.iMax));
    float outL = gCalib.kp * eL + _phaseIL + gCalib.kd * (eL - _phasePrevEL) / dt;

    // PID roue droite
    _phaseIR = fmaxf(-gCalib.iMax, fminf(_phaseIR + gCalib.ki * eR * dt, gCalib.iMax));
    float outR = gCalib.kp * eR + _phaseIR + gCalib.kd * (eR - _phasePrevER) / dt;

    _phasePrevEL = eL;
    _phasePrevER = eR;

    // Profil de freinage : cible v=0 à d=stopMm
    float dL = fmaxf(0.0f, fabsf(eL) - _phaseStopMm);
    float dR = fmaxf(0.0f, fabsf(eR) - _phaseStopMm);
    float brakingL = sqrtf(2.0f * _phaseAccel * dL);
    float brakingR = sqrtf(2.0f * _phaseAccel * dR);
    float capL = fminf(brakingL, _phaseSpeedCap);
    float capR = fminf(brakingR, _phaseSpeedCap);

    outL = fmaxf(-capL, fminf(outL, capL));
    outR = fmaxf(-capR, fminf(outR, capR));

    if (fabsf(outL) < gCalib.minSpd) outL = copysignf(gCalib.minSpd, eL);
    if (fabsf(outR) < gCalib.minSpd) outR = copysignf(gCalib.minSpd, eR);

    _motion->setMotorVelocities(outL, outR);
    _lastCmdL = outL;
    _lastCmdR = outR;
    return false;
}

// ─── Contrôleur pose-based (phase TRANSLATE) ─────────────────────────────────
//
//  Au lieu de viser des cibles wheel-position, on calcule directement les
//  vitesses linéaire et angulaire à partir de l'erreur de pose, puis on les
//  applique via cinématique différentielle :
//
//    v_linear  = clamp(speedCap, braking_profile) × cos(heading_err)
//    v_angular = Kp_ang × heading_err
//    vL = v_linear − v_angular × wheelbase/2
//    vR = v_linear + v_angular × wheelbase/2
//
//  Avantages :
//  • Vrai blend sans approximation : la commande s'adapte continuellement
//  • Pas de résidu géométrique (converge sur dist géométrique directement)
//  • Pour grand angle, cos(err)→0 ⇒ rotation pure en place, naturellement
//
bool MotionController::_runPoseStep() {
    const float dt = OBS_POLL_MS / 1000.0f;

    // Le blend vise un suivi de trajectoire rapide, PAS la précision waypoint.
    // Tolérances larges pour éviter les oscillations en fin de phase :
    constexpr float BLEND_ARRIVAL_MM  = 25.0f;   // zone "arrivé" — phase done
    constexpr float BLEND_APPROACH_MM = 60.0f;   // sous ce seuil : pas de correction angulaire
                                                  // (évite atan2 chaotique près du point)

    const float curX = gDisplay.enc_pose_x_mm;
    const float curY = gDisplay.enc_pose_y_mm;
    const float curTheta = gDisplay.enc_pose_theta_rad;

    const float dx = _activeTgt.x_mm - curX;
    const float dy = _activeTgt.y_mm - curY;
    const float dist = sqrtf(dx*dx + dy*dy);

    _distMm = dist;
    gDisplay.nav_dist_mm = dist;

    // Convergence avec tolérance large (pas de précision en blend)
    if (dist <= BLEND_ARRIVAL_MM) {
        _motion->softStop();
        _lastCmdL = 0; _lastCmdR = 0;
        return true;
    }

    // Ramp speedCap + profil de freinage
    _phaseSpeedCap = fminf(_phaseSpeedCap + _phaseAccel * dt, _phaseSpeed);
    const float brakingCap = sqrtf(2.0f * _phaseAccel * fmaxf(0.0f, dist - BLEND_ARRIVAL_MM));
    const float v_linear_max = fminf(_phaseSpeedCap, brakingCap);

    // Direction vers la cible
    float thetaToTarget = atan2f(-dy, dx);
    if (_activeGoBack) thetaToTarget = _normAngle(thetaToTarget + PI_F);
    const float headingErr = _normAngle(thetaToTarget - curTheta);
    gDisplay.nav_delta_deg = headingErr * RAD2DEG;

    // Vitesse linéaire = v_max × cos(headingErr)
    const float align_factor = fmaxf(0.0f, cosf(headingErr));
    float v_linear = v_linear_max * align_factor;
    if (_activeGoBack) v_linear = -v_linear;
    if (align_factor > 0.5f && fabsf(v_linear) < gCalib.minSpd) {
        v_linear = copysignf(gCalib.minSpd, _activeGoBack ? -1.0f : 1.0f);
    }

    // Vitesse angulaire : coupée en zone d'approche pour éviter le frétillage
    float v_angular = 0.0f;
    if (dist > BLEND_APPROACH_MM) {
        const float Kp_ang = 4.0f;
        const float omega_max = 6.0f;
        v_angular = Kp_ang * headingErr;
        v_angular = fmaxf(-omega_max, fminf(v_angular, omega_max));
    }

    // Cinématique différentielle
    const float half_b = gCalib.encWheelbase * 0.5f;
    float vL = v_linear - v_angular * half_b;
    float vR = v_linear + v_angular * half_b;
    vL = fmaxf(-_phaseSpeed, fminf(vL, _phaseSpeed));
    vR = fmaxf(-_phaseSpeed, fminf(vR, _phaseSpeed));

    _motion->setMotorVelocities(vL, vR);
    _lastCmdL = vL;
    _lastCmdR = vR;

    return false;
}

// ─── Détection obstacle (port de Robot) ──────────────────────────────────────

bool MotionController::_obstacleInDir(float dir_rad) {
    switch (_detectMode) {
        case DetectMode::WALL_FILTERED: return _obstacleWallFiltered(dir_rad);
        default:                         return _obstacleSimple(dir_rad);
    }
}

bool MotionController::_obstacleSimple(float dir_rad) {
    uint16_t n = _lidar->getScan(_scanBuf, LD06_SCAN_BUF_SIZE);
    if (n == 0) return false;

    float rTheta = gDisplay.enc_pose_theta_rad;
    float minFwd = OBS_DETECT_DIST_MM;
    float minAngle = 0;
    bool  found = false;

    for (uint16_t i = 0; i < n; i++) {
        if (_scanBuf[i].distance_mm == 0) continue;
        if (_scanBuf[i].confidence < OBS_CONFIDENCE_MIN) continue;
        float d = (float)_scanBuf[i].distance_mm;
        if (d < LIDAR_BODY_DIST_MM) continue;

        float rDeg = fmodf(270.0f - _scanBuf[i].angle_deg + gCalib.lidarOffsetDeg + 360.0f, 360.0f);
        if ((rDeg >= LIDAR_BLIND_L_START && rDeg <= LIDAR_BLIND_L_END) ||
            (rDeg >= LIDAR_BLIND_R_START && rDeg <= LIDAR_BLIND_R_END)) continue;

        float lidarAngle = (270.0f - _scanBuf[i].angle_deg + gCalib.lidarOffsetDeg) * DEG2RAD;
        float px = d * cosf(lidarAngle);
        float py = d * sinf(lidarAngle);

        float delta = dir_rad - rTheta;
        float fwd   =  px * cosf(delta) + py * sinf(delta);
        float lat   = -px * sinf(delta) + py * cosf(delta);

        if (fwd > OBS_MIN_DIST_MM && fwd < OBS_DETECT_DIST_MM &&
            fabsf(lat) < OBS_WIDTH_MM * 0.5f) {
            if (fwd < minFwd) {
                minFwd   = fwd;
                minAngle = atan2f(py, px) * RAD2DEG;
            }
            found = true;
        }
    }

    if (found) {
        gDisplay.obs_dist_mm   = minFwd;
        gDisplay.obs_angle_deg = minAngle;
    }
    return found;
}

bool MotionController::_obstacleWallFiltered(float dir_rad) {
    uint16_t n = _lidar->getScan(_scanBuf, LD06_SCAN_BUF_SIZE);
    if (n == 0) return false;

    float rTheta = gDisplay.enc_pose_theta_rad;

    for (uint16_t i = 0; i < n; i++) {
        if (_scanBuf[i].distance_mm == 0) continue;
        if (_scanBuf[i].confidence < OBS_CONFIDENCE_MIN) continue;
        float d = (float)_scanBuf[i].distance_mm;
        if (d < LIDAR_BODY_DIST_MM) continue;

        float rDeg = fmodf(270.0f - _scanBuf[i].angle_deg + gCalib.lidarOffsetDeg + 360.0f, 360.0f);
        if ((rDeg >= LIDAR_BLIND_L_START && rDeg <= LIDAR_BLIND_L_END) ||
            (rDeg >= LIDAR_BLIND_R_START && rDeg <= LIDAR_BLIND_R_END)) continue;

        float wx, wy;
        _lidarToWorld(_scanBuf[i].angle_deg, d, wx, wy);
        if (wx < TABLE_MARGIN_MM || wx > TABLE_WIDTH_MM  - TABLE_MARGIN_MM) continue;
        if (wy < TABLE_MARGIN_MM || wy > TABLE_HEIGHT_MM - TABLE_MARGIN_MM) continue;

        float lidarAngle = (270.0f - _scanBuf[i].angle_deg + gCalib.lidarOffsetDeg) * DEG2RAD;
        float px = d * cosf(lidarAngle);
        float py = d * sinf(lidarAngle);

        float delta = dir_rad - rTheta;
        float fwd   =  px * cosf(delta) + py * sinf(delta);
        float lat   = -px * sinf(delta) + py * cosf(delta);

        if (fwd > OBS_MIN_DIST_MM && fwd < OBS_DETECT_DIST_MM &&
            fabsf(lat) < OBS_WIDTH_MM * 0.5f) {
            return true;
        }
    }
    return false;
}

void MotionController::_lidarToWorld(float angle_deg, float dist_mm,
                                      float& wx, float& wy) const {
    float a = (270.0f - angle_deg + gCalib.lidarOffsetDeg) * DEG2RAD
              + gDisplay.enc_pose_theta_rad;
    wx = gDisplay.enc_pose_x_mm + dist_mm * cosf(a);
    wy = gDisplay.enc_pose_y_mm - dist_mm * sinf(a);
}

float MotionController::_normAngle(float rad) {
    while (rad >  PI_F) rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
}
