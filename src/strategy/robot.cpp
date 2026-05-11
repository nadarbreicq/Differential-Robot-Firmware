#include "robot.h"
#include "../display/oled.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "ROBOT";

static constexpr float PI_F = 3.14159265f;
static constexpr float DEG2RAD = PI_F / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI_F;

Robot::Robot(StepControl &motion, LD06 &lidar)
    : _motion(motion), _lidar(lidar) {}

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

        ESP_LOGW(TAG, "Obstacle %.0fmm %+.0fdeg — recul, %.0fmm restants",
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

bool Robot::goStall(float mm, uint32_t timeoutMs) {
    if (fabsf(mm) < 0.5f) return false;

    _motion.startGo(mm);

    int32_t prevL = _encLeft  ? _encLeft->getCount()  : 0;
    int32_t prevR = _encRight ? _encRight->getCount() : 0;
    uint8_t  stallCount = 0;
    uint32_t start      = millis();

    while (_motion.isMoving()) {
        vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));
        _motion.syncPose();

        int32_t curL  = _encLeft  ? _encLeft->getCount()  : prevL;
        int32_t curR  = _encRight ? _encRight->getCount() : prevR;
        int32_t dL    = abs(curL - prevL);
        int32_t dR    = abs(curR - prevR);
        prevL = curL;
        prevR = curR;

        // Seuil : < 0.1 mm par poll sur les deux roues = pas de mouvement
        constexpr int32_t kThresh = (int32_t)(0.1f / MM_PER_COUNT) + 1;

        if (dL < kThresh && dR < kThresh) {
            if (++stallCount >= 3) {
                _motion.stop();
                _motion.syncPose();
                return true;
            }
        } else {
            stallCount = 0;
        }

        if (millis() - start > timeoutMs) {
            _motion.stop();
            _motion.syncPose();
            return false;
        }

        if (isMatchOver()) { _motion.stop(); disableMotors(); gDisplay.robot_state = RobotState::DONE; return false; }
    }

    _motion.syncPose();
    return false;
}

// ─── Boucle PID commune (double PID position, une boucle par roue) ───────────

void Robot::_runWheelPID(int32_t tL, int32_t tR, float speed, float accel, float stopMm) {
    _motion.setAcceleration(accel);

    float iL = 0, iR = 0;
    float prevEL = (float)(tL - _encLeft->getCount())  * MM_PER_COUNT;
    float prevER = (float)(tR - _encRight->getCount()) * MM_PER_COUNT;
    const float dt = OBS_POLL_MS / 1000.0f;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));

        float eL  = (float)(tL - _encLeft->getCount())  * MM_PER_COUNT;
        float eR  = (float)(tR - _encRight->getCount()) * MM_PER_COUNT;
        float avg = (fabsf(eL) + fabsf(eR)) * 0.5f;

        gDisplay.nav_dist_mm   = avg;
        gDisplay.nav_delta_deg = eL - eR;

        if (avg <= stopMm) { _motion.stop(); break; }

        // PID roue gauche
        iL = fmaxf(-ENC_P1_I_MAX, fminf(iL + ENC_P1_KI * eL * dt, ENC_P1_I_MAX));
        float outL = ENC_P1_KP * eL + iL + ENC_P1_KD * (eL - prevEL) / dt;

        // PID roue droite
        iR = fmaxf(-ENC_P1_I_MAX, fminf(iR + ENC_P1_KI * eR * dt, ENC_P1_I_MAX));
        float outR = ENC_P1_KP * eR + iR + ENC_P1_KD * (eR - prevER) / dt;

        prevEL = eL;
        prevER = eR;

        outL = fmaxf(-speed, fminf(outL, speed));
        outR = fmaxf(-speed, fminf(outR, speed));

        if (fabsf(outL) < ENC_P1_MIN_SPD) outL = copysignf(ENC_P1_MIN_SPD, outL);
        if (fabsf(outR) < ENC_P1_MIN_SPD) outR = copysignf(ENC_P1_MIN_SPD, outR);

        _motion.setMotorVelocities(outL, outR);

        if (isMatchOver())                { _motion.stop(); disableMotors(); gDisplay.robot_state = RobotState::DONE; return; }
        if (!_nearEndMode && isEndgame()) { _motion.softStop(OBS_STOP_ACCEL_MMS2); break; }
        if (_obstacleEn && _obstacleInDir(_motion.getTheta())) {
            _motion.softStop(OBS_STOP_ACCEL_MMS2);
            gDisplay.robot_state = RobotState::OBSTACLE;
            _waitObstacleClear(_motion.getTheta());
            // Réinitialise le PID pour éviter un spike de dérivée à la reprise
            iL = iR = 0;
            prevEL = (float)(tL - _encLeft->getCount())  * MM_PER_COUNT;
            prevER = (float)(tR - _encRight->getCount()) * MM_PER_COUNT;
            gDisplay.robot_state = RobotState::GOTO;
            continue;   // reprend vers la même cible tL/tR
        }
    }

    // Synchronise la pose dead reckoning (pas) sur la pose encodeurs.
    // Sans ça, _motion._theta reste à sa valeur d'avant le mouvement PID,
    // et les syncPose() suivants (go, goStall) calculent les deltas avec le
    // mauvais angle.
    vTaskDelay(pdMS_TO_TICKS(10));   // laisse taskEncoders faire un tick
    _motion.setPosition(gDisplay.enc_pose_x_mm,
                        gDisplay.enc_pose_y_mm,
                        gDisplay.enc_pose_theta_deg);
}

// ─── Translation asservie encodeurs ──────────────────────────────────────────

void Robot::goPID(float mm) {
    if (fabsf(mm) < 0.5f) return;
    if (isMatchOver() || (!_nearEndMode && isEndgame())) return;
    gDisplay.robot_state = RobotState::MOVING;

    int32_t counts = (int32_t)(fabsf(mm) / MM_PER_COUNT);
    int32_t sign   = (mm >= 0) ? 1 : -1;
    _runWheelPID(_encLeft->getCount()  + sign * counts,
                 _encRight->getCount() + sign * counts,
                 _motion.getSpeed(), _motion.getAcceleration());

    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;
}

// ─── Rotation asservie encodeurs ─────────────────────────────────────────────

void Robot::turnPID(float deg) {
    if (fabsf(deg) < 0.5f) return;
    if (isMatchOver() || (!_nearEndMode && isEndgame())) return;
    gDisplay.robot_state = RobotState::TURNING;

    // Arc parcouru par chaque roue codeuse pour une rotation de |deg|
    float arc  = ENC_WHEELBASE_MM * PI_F * fabsf(deg) / 360.0f;
    float sign = (deg >= 0) ? 1.0f : -1.0f;   // > 0 = CCW = gauche

    int32_t tL = _encLeft->getCount()  - (int32_t)(sign * arc / MM_PER_COUNT);
    int32_t tR = _encRight->getCount() + (int32_t)(sign * arc / MM_PER_COUNT);

    // Détection obstacle désactivée pendant la rotation :
    // la direction de détection serait fausse (theta dead reckoning figé)
    // et le LIDAR peut croiser des obstacles latéraux sans danger.
    bool obsWas = _obstacleEn;
    _obstacleEn = false;

    float stopMm  = ENC_P1_STOP_DEG * ENC_WHEELBASE_MM * PI_F / 360.0f;
    float turnSpd = fminf(_motion.getSpeed(), TURN_SPEED_MMS);
    float turnAcc = fminf(_motion.getAcceleration(), TURN_ACCEL_MMS2);
    _runWheelPID(tL, tR, turnSpd, turnAcc, stopMm);

    _obstacleEn = obsWas;
    gDisplay.nav_dist_mm = 0;
    gDisplay.robot_state = RobotState::IDLE;
}

// ─── Navigation odométrie encodeurs — double PID position ────────────────────

void Robot::gotoXYenc(float tx, float ty) {
    gotoXYenc(tx, ty, NAN);
}

void Robot::gotoXYenc(float tx, float ty, float arrival_deg) {
    gDisplay.robot_state = RobotState::GOTO;

    float savedSpeed = _motion.getSpeed();
    float savedAccel = _motion.getAcceleration();

    // Distance calculée une seule fois — pas de recalcul après rotation
    const float dx   = tx - gDisplay.enc_pose_x_mm;
    const float dy   = ty - gDisplay.enc_pose_y_mm;
    const float dist = sqrtf(dx*dx + dy*dy);

    // Garde minimale : évite atan2(0,0) — le PID gère l'arrêt précis via stopMm
    if (dist > 1.0f) {
        // ── 1. Turn vers la cible ────────────────────────────────────────
        float aerr    = _normAngle(atan2f(-dy, dx) - gDisplay.enc_pose_theta_rad);
        bool  goBack  = fabsf(aerr) > PI_F * 0.5f;   // >90° → reculer plus court
        float turnErr = goBack ? _normAngle(aerr + (aerr >= 0 ? -PI_F : PI_F)) : aerr;

        if (fabsf(turnErr) > 0.05f)   // < 3° : pas de pré-alignement
            turnPID(turnErr * RAD2DEG);

        // ── 2. Go (counts capturés après la rotation) ────────────────────
        int32_t counts = (int32_t)(dist / MM_PER_COUNT);
        int32_t sign   = goBack ? -1 : 1;
        _runWheelPID(_encLeft->getCount()  + sign * counts,
                     _encRight->getCount() + sign * counts,
                     savedSpeed, savedAccel);
    }

    _motion.setSpeed(savedSpeed);
    _motion.setAcceleration(savedAccel);
    gDisplay.nav_dist_mm = 0;

    // ── 3. Turn final — uniquement si demandé (seuil géré par turnPID ≥ 0.5°)
    if (!isnanf(arrival_deg)) {
        float delta = _normAngle(arrival_deg * DEG2RAD - gDisplay.enc_pose_theta_rad);
        turnPID(delta * RAD2DEG);   // turnPID ignore si |deg| < 0.5°
    }

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
    ESP_LOGI(TAG, "Position: x=%.0f y=%.0f θ=%.1f°", x_mm, y_mm, theta_deg);
}

// ─── Cinématique ──────────────────────────────────────────────────────────────

void Robot::setSpeed(float mmS) {
    _motion.setSpeed(mmS);
}

void Robot::setSpeedPct(float speedPct, float accelPct) {
    if (accelPct < 0.0f) accelPct = speedPct;   // même % si non spécifié
    _motion.setSpeed(DEFAULT_SPEED_MMS       * speedPct / 100.0f);
    _motion.setAcceleration(DEFAULT_ACCEL_MMS2 * accelPct / 100.0f);
}

void Robot::resetSpeed() {
    _motion.setSpeed(DEFAULT_SPEED_MMS);
    _motion.setAcceleration(DEFAULT_ACCEL_MMS2);
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
        float rDeg = fmodf(270.0f - _scanBuf[i].angle_deg + LIDAR_OFFSET_DEG + 360.0f, 360.0f);
        if ((rDeg >= LIDAR_BLIND_L_START && rDeg <= LIDAR_BLIND_L_END) ||
            (rDeg >= LIDAR_BLIND_R_START && rDeg <= LIDAR_BLIND_R_END)) continue;

        float lidarAngle = (270.0f - _scanBuf[i].angle_deg + LIDAR_OFFSET_DEG) * DEG2RAD;
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
        float rDeg = fmodf(270.0f - _scanBuf[i].angle_deg + LIDAR_OFFSET_DEG + 360.0f, 360.0f);
        if ((rDeg >= LIDAR_BLIND_L_START && rDeg <= LIDAR_BLIND_L_END) ||
            (rDeg >= LIDAR_BLIND_R_START && rDeg <= LIDAR_BLIND_R_END)) continue;

        float wx, wy;
        _lidarToWorld(_scanBuf[i].angle_deg, d, wx, wy);
        if (wx < TABLE_MARGIN_MM || wx > TABLE_WIDTH_MM  - TABLE_MARGIN_MM) continue;
        if (wy < TABLE_MARGIN_MM || wy > TABLE_HEIGHT_MM - TABLE_MARGIN_MM) continue;

        float lidarAngle = (270.0f - _scanBuf[i].angle_deg + LIDAR_OFFSET_DEG) * DEG2RAD;
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
    float a   = (270.0f - angle_deg + LIDAR_OFFSET_DEG) * DEG2RAD + _motion.getTheta();
    wx = _motion.getX() + dist_mm * cosf(a);
    wy = _motion.getY() - dist_mm * sinf(a);   // Y+ vers le bas → sin inversé
}

void Robot::_waitObstacleClear(float dir_rad) {
    vTaskDelay(pdMS_TO_TICKS(500));   // attente minimale avant de re-sonder
    uint32_t start = millis();
    while (_obstacleInDir(dir_rad)) {
        if (millis() - start > OBS_WAIT_MS) {
            ESP_LOGW(TAG, "Timeout obstacle — reprise quand même");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Obstacle dégagé — reprise");
}

float Robot::_normAngle(float rad) {
    while (rad >  PI_F) rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
}
