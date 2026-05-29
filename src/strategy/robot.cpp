#include "robot.h"
#include "../display/oled.h"
#include "../live_config.h"
#include "../log.h"
#include <math.h>

static constexpr float PI_F = 3.14159265f;
static constexpr float DEG2RAD = PI_F / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI_F;

Robot::Robot(StepControl &motion, LD06 &lidar, MotionController &motionCtrl)
    : _motion(motion), _lidar(lidar), _motionCtrl(motionCtrl) {}

// ─── Chrono de match ──────────────────────────────────────────────────────────

void Robot::startMatch() {
    gDisplay.match_start_ms = millis();
}

uint32_t Robot::matchElapsed() const {
    if (gDisplay.match_start_ms == 0) return 0;
    return millis() - gDisplay.match_start_ms;
}

bool Robot::isEndgame()   const { return matchElapsed() >= MATCH_ENDGAME_MS;  }
bool Robot::isMatchOver() const { return matchElapsed() >= MATCH_DURATION_MS; }

void Robot::waitMatchTime(uint32_t target_ms) {
    while (!isMatchOver() && matchElapsed() < target_ms)
        vTaskDelay(pdMS_TO_TICKS(10));
}

// ─── Déplacements ─────────────────────────────────────────────────────────────

void Robot::go(float mm) {
    if (fabsf(mm) < 0.5f) return;

    float remaining  = mm;
    float savedSpeed = _motion.getSpeed();
    float savedAccel = _motion.getAcceleration();

    bool endgameFired = false;

    while (fabsf(remaining) > 0.5f && !endgameFired) {
        gDisplay.robot_state = RobotState::MOVING;
        float dir = _motion.getTheta() + (remaining < 0 ? PI_F : 0.0f);
        float x0  = _motion.getX();
        float y0  = _motion.getY();

        _motion.setSpeed(savedSpeed);
        _motion.setAcceleration(savedAccel);

        float sign   = (remaining >= 0) ? 1.0f : -1.0f;
        bool  obsHit = false;

        _motion.startGo(remaining);

        while (_motion.isMoving()) {
            vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));
            _motion.syncPose();

            if (isMatchOver())                     { _motion.softStop(OBS_STOP_ACCEL_MMS2); disableMotors(); gDisplay.robot_state = RobotState::DONE; return; }
            if (!_nearEndMode && isEndgame())      { _motion.softStop(OBS_STOP_ACCEL_MMS2); endgameFired = true; break; }
            if (_obstacleEn && _obstacleInDir(dir)) { _motion.softStop(OBS_STOP_ACCEL_MMS2); obsHit = true; break; }
        }

        float dx      = _motion.getX() - x0;
        float dy      = _motion.getY() - y0;
        float traveled = sign * sqrtf(dx*dx + dy*dy);
        remaining     -= traveled;

        _motion.syncPose();

        if (!obsHit) return;

        LOG_W("ROBOT", "Obstacle %.0fmm %+.0fdeg — recul, %.0fmm restants",
              (double)gDisplay.obs_dist_mm, (double)gDisplay.obs_angle_deg, (double)remaining);

        _motion.setSpeed(savedSpeed);
        _motion.setAcceleration(savedAccel);
        _motion.go(-sign * OBS_BACKUP_MM);
        remaining += OBS_BACKUP_MM;

        gDisplay.robot_state = RobotState::OBSTACLE;
        _waitObstacleClear(dir);
    }

    _motion.setSpeed(savedSpeed);
    _motion.setAcceleration(savedAccel);
}

// ─── Go avec détection de blocage (plaquage bordure) ─────────────────────────
// Démarre le mouvement et surveille les encodeurs. Si les deux roues cessent de
// varier pendant STALL_POLLS polls consécutifs → robot bloqué → stop.
// Retourne true si stall détecté, false si distance atteinte ou timeout.

bool Robot::goStall(float mm, uint32_t timeoutMs, uint32_t stallConfirmMs) {
    if (fabsf(mm) < 0.5f) return false;

    // Direction de déplacement (même logique que go() pour l'obstacle)
    const float moveDir = _motion.getTheta() + ((mm < 0) ? PI_F : 0.0f);
    const float sign    = (mm >= 0) ? 1.0f : -1.0f;

    _motion.startGo(mm);

    int32_t  prevL        = _encLeft  ? _encLeft->getCount()  : 0;
    int32_t  prevR        = _encRight ? _encRight->getCount() : 0;
    float    traveled     = 0.0f;
    uint32_t stallSince   = 0;
    uint32_t timeoutStart = 0;

    const int32_t kThresh = (int32_t)(STALL_THRESH_MM / gCalib.mmPerCount) + 1;

    while (_motion.isMoving()) {
        vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));
        _motion.syncPose();

        int32_t curL = _encLeft  ? _encLeft->getCount()  : prevL;
        int32_t curR = _encRight ? _encRight->getCount() : prevR;
        int32_t dL   = abs(curL - prevL);
        int32_t dR   = abs(curR - prevR);
        prevL = curL;
        prevR = curR;

        uint32_t now = millis();
        traveled += (float)(dL + dR) * 0.5f * gCalib.mmPerCount;

        if (timeoutStart == 0 && traveled >= fabsf(mm))
            timeoutStart = now;

        // Détection obstacle dans le sens du mouvement (adversaire, pas le mur cible)
        // Le mur cible est à <LIDAR_BODY_DIST_MM quand plaqué → jamais déclenché
        if (_obstacleEn && _obstacleInDir(moveDir)) {
            _motion.softStop(OBS_STOP_ACCEL_MMS2);
            gDisplay.robot_state = RobotState::OBSTACLE;
            _waitObstacleClear(moveDir);
            stallSince = 0;
            float remaining = fabsf(mm) - traveled;
            if (remaining > 0.5f) _motion.startGo(sign * remaining);
            prevL = _encLeft  ? _encLeft->getCount()  : prevL;
            prevR = _encRight ? _encRight->getCount() : prevR;
            gDisplay.robot_state = RobotState::MOVING;
            continue;
        }

        // Détection stall : active dès le début (mur plus proche que prévu aussi)
        if (dL < kThresh && dR < kThresh) {
            if (stallSince == 0) stallSince = now;
            if (now - stallSince >= stallConfirmMs) {
                _motion.stop();
                _motion.syncPose();
                return true;
            }
        } else {
            stallSince = 0;
        }

        // Timeout : seulement après avoir atteint la position normale
        if (timeoutStart != 0 && now - timeoutStart >= timeoutMs) {
            _motion.stop();
            _motion.syncPose();
            return false;
        }

        if (isMatchOver()) { _motion.stop(); disableMotors(); gDisplay.robot_state = RobotState::DONE; return false; }
    }

    _motion.syncPose();
    return false;
}

// ─── API motion non-bloquante (wrappers vers MotionController) ───────────────

bool Robot::waitArrived(float blendMm) {
    // Capturer le seq utilisateur AVANT d'attendre : c'est la cible qu'on suit.
    // Sans ça, l'état IDLE initial (avant que la tâche motion n'ait vu le seq)
    // ferait retourner waitArrived immédiatement.
    uint32_t expected = _motionCtrl.getUserSeq();
    while (true) {
        if (isMatchOver()) {
            _motionCtrl.release();
            _motion.stop();
            disableMotors();
            gDisplay.robot_state = RobotState::DONE;
            return false;
        }
        if (!_nearEndMode && isEndgame()) {
            _motionCtrl.release();
            return false;
        }
        if (_motionCtrl.getUserSeq() != expected) return false;     // préempté
        if (_motionCtrl.getDoneSeq() == expected) return true;      // terminé
        if (blendMm > 0.0f && _motionCtrl.getDistMm() < blendMm) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Robot::setTarget(float x_mm, float y_mm) {
    setTarget(x_mm, y_mm, NAN, false);
}

void Robot::setTarget(float x_mm, float y_mm, float arrival_deg, bool backward) {
    MotionController::Target t;
    t.x_mm       = x_mm;
    t.y_mm       = y_mm;
    t.theta_deg  = arrival_deg;
    t.speed      = _motion.getSpeed();
    t.accel      = _motion.getAcceleration();
    t.stopMm     = gCalib.stopMm;
    t.backward   = backward;
    t.obstacleEn = _obstacleEn;
    _motionCtrl.setTarget(t);
}

// ─── Translation asservie encodeurs ──────────────────────────────────────────

void Robot::goPID(float mm) {
    if (fabsf(mm) < 0.5f) return;
    if (isMatchOver() || (!_nearEndMode && isEndgame())) return;
    gDisplay.robot_state = RobotState::MOVING;

    // Cible XY = pose courante + (mm) dans la direction theta
    float curX = gDisplay.enc_pose_x_mm;
    float curY = gDisplay.enc_pose_y_mm;
    float curT = gDisplay.enc_pose_theta_rad;
    float sign = (mm >= 0) ? 1.0f : -1.0f;
    float d    = fabsf(mm);
    float tx   = curX + sign * d * cosf(curT);
    float ty   = curY - sign * d * sinf(curT);   // Y+ vers le bas

    setTarget(tx, ty, NAN, mm < 0);
    waitArrived(0);

    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;
}

// ─── Rotation asservie encodeurs ─────────────────────────────────────────────

void Robot::turnPID(float deg) {
    if (fabsf(deg) < 0.5f) return;
    if (isMatchOver() || (!_nearEndMode && isEndgame())) return;
    gDisplay.robot_state = RobotState::TURNING;

    // Cible : pose courante + delta theta (TRANSLATE skip car dist=0)
    float curX = gDisplay.enc_pose_x_mm;
    float curY = gDisplay.enc_pose_y_mm;
    float curThetaDeg = gDisplay.enc_pose_theta_deg;
    setTarget(curX, curY, curThetaDeg + deg, false);
    waitArrived(0);

    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;
}

// ─── Navigation odométrie encodeurs — wrapper ────────────────────────────────

void Robot::gotoXYenc(float tx, float ty) {
    gotoXYenc(tx, ty, NAN);
}

void Robot::gotoXYenc(float tx, float ty, float arrival_deg, bool backward) {
    if (isMatchOver() || (!_nearEndMode && isEndgame())) return;
    gDisplay.robot_state = RobotState::GOTO;
    setTarget(tx, ty, arrival_deg, backward);
    waitArrived(0);
    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;
}

void Robot::turn(float deg) {
    if (fabsf(deg) < 0.5f) return;
    if (isMatchOver() || (!_nearEndMode && isEndgame())) return;
    gDisplay.robot_state = RobotState::TURNING;
    _motion.turn(deg);
    gDisplay.robot_state = RobotState::IDLE;
}

void Robot::gotoXY(float tx, float ty) {
    gotoXY(tx, ty, NAN);
}

void Robot::gotoXY(float tx, float ty, float arrival_deg) {
    gDisplay.robot_state = RobotState::GOTO;
    float dx = tx - _motion.getX();
    float dy = ty - _motion.getY();
    float dist = sqrtf(dx * dx + dy * dy);

    float target_angle = atan2f(-dy, dx);
    float delta        = _normAngle(target_angle - _motion.getTheta());
    gDisplay.nav_delta_deg = delta * RAD2DEG;
    gDisplay.nav_dist_mm   = dist;

    if (dist < 1.0f) goto arrival;

    {
        // 1. Tourne vers la cible
        turn(delta * RAD2DEG);

        // 2. Avance jusqu'à la cible
        go(dist);
    }

arrival:
    // 3. Rotation vers l'angle d'arrivée (optionnel)
    if (!isnanf(arrival_deg)) {
        float target_rad = arrival_deg * DEG2RAD;
        float delta = _normAngle(target_rad - _motion.getTheta());
        if (fabsf(delta) > 0.01f)
            turn(delta * RAD2DEG);
    }
    gDisplay.nav_dist_mm = 0;   // efface l'affichage nav
}

// ─── Pose ─────────────────────────────────────────────────────────────────────

void Robot::setPosition(float x_mm, float y_mm, float theta_deg) {
    _motion.setPosition(x_mm, y_mm, theta_deg);
    // Synchronise l'odométrie encodeurs sur la même position
    gDisplay.enc_reset_x         = x_mm;
    gDisplay.enc_reset_y         = y_mm;
    gDisplay.enc_reset_theta_deg = theta_deg;
    gDisplay.enc_reset_pending   = true;
    while (gDisplay.enc_reset_pending) vTaskDelay(1);   // attend que taskEncoders applique
    LOG_I("ROBOT", "Position: x=%.0f y=%.0f theta=%.1f", x_mm, y_mm, theta_deg);
}

// ─── Cinématique ──────────────────────────────────────────────────────────────

void Robot::setSpeed(float mmS) {
    _motion.setSpeed(mmS);
}

void Robot::setSpeedPct(float speedPct, float accelPct) {
    if (accelPct < 0.0f) accelPct = speedPct;   // même % si non spécifié
    _motion.setSpeed(gCalib.defaultSpeed * speedPct / 100.0f);
    _motion.setAcceleration(gCalib.defaultAccel * accelPct / 100.0f);
}

void Robot::resetSpeed() {
    _motion.setSpeed(gCalib.defaultSpeed);
    _motion.setAcceleration(gCalib.defaultAccel);
}

// ─── Obstacle ─────────────────────────────────────────────────────────────────

bool Robot::_obstacleInDir(float dir_rad) {
    switch (_detectMode) {
        case DetectMode::WALL_FILTERED: return _obstacleWallFiltered(dir_rad);
        default:                        return _obstacleSimple(dir_rad);
    }
}

// Détection simple : rectangle devant/derrière, pas de filtre murs.
bool Robot::_obstacleSimple(float dir_rad) {
    uint16_t n = _lidar.getScan(_scanBuf, LD06_SCAN_BUF_SIZE);
    if (n == 0) return false;

    float rTheta = _motion.getTheta();
    float minFwd = OBS_DETECT_DIST_MM;
    float minAngle = 0;
    bool  found  = false;

    for (uint16_t i = 0; i < n; i++) {
        if (_scanBuf[i].distance_mm == 0) continue;
        if (_scanBuf[i].confidence < OBS_CONFIDENCE_MIN) continue;
        float d = (float)_scanBuf[i].distance_mm;
        if (d < LIDAR_BODY_DIST_MM) continue;

        // Zones aveugles (poteaux structurels)
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

// Détection avec filtre murs : ignore les points identifiés comme bordures.
// Requiert une pose robot correcte (après calage bordure).
bool Robot::_obstacleWallFiltered(float dir_rad) {
    uint16_t n = _lidar.getScan(_scanBuf, LD06_SCAN_BUF_SIZE);
    if (n == 0) return false;

    float rTheta = _motion.getTheta();

    for (uint16_t i = 0; i < n; i++) {
        if (_scanBuf[i].distance_mm == 0) continue;
        if (_scanBuf[i].confidence < OBS_CONFIDENCE_MIN) continue;
        float d = (float)_scanBuf[i].distance_mm;
        if (d < LIDAR_BODY_DIST_MM) continue;

        // Zones aveugles (poteaux structurels)
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

void Robot::_lidarToWorld(float angle_deg, float dist_mm,
                          float &wx, float &wy) const {
    float a   = (270.0f - angle_deg + gCalib.lidarOffsetDeg) * DEG2RAD + _motion.getTheta();
    wx = _motion.getX() + dist_mm * cosf(a);
    wy = _motion.getY() - dist_mm * sinf(a);   // Y+ vers le bas → sin inversé
}

void Robot::_waitObstacleClear(float dir_rad) {
    vTaskDelay(pdMS_TO_TICKS(500));   // attente minimale avant de re-sonder
    uint32_t start = millis();
    while (_obstacleInDir(dir_rad)) {
        if (millis() - start > OBS_WAIT_MS) {
            LOG_W("ROBOT", "Timeout obstacle — reprise quand meme");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    LOG_I("ROBOT", "Obstacle degage — reprise");
}

float Robot::_normAngle(float rad) {
    while (rad >  PI_F) rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
}
