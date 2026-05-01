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

// ─── Déplacements ─────────────────────────────────────────────────────────────

void Robot::go(float mm) {
    if (fabsf(mm) < 0.5f) return;

    float remaining  = mm;
    float savedSpeed = _motion.getSpeed();
    float savedAccel = _motion.getAcceleration();

    while (fabsf(remaining) > 0.5f) {
        gDisplay.robot_state = RobotState::MOVING;
        float dir = _motion.getTheta() + (remaining < 0 ? PI_F : 0.0f);
        float x0  = _motion.getX();
        float y0  = _motion.getY();

        _motion.setSpeed(savedSpeed);
        _motion.setAcceleration(savedAccel);
        _motion.startGo(remaining);

        bool obsHit = false;
        while (_motion.isMoving()) {
            vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));
            if (_obstacleEn && _obstacleInDir(dir)) {
                _motion.softStop(OBS_STOP_ACCEL_MMS2);   // freinage agressif
                obsHit = true;
                break;
            }
        }

        float dx      = _motion.getX() - x0;
        float dy      = _motion.getY() - y0;
        float sign    = (remaining >= 0) ? 1.0f : -1.0f;
        float traveled = sign * sqrtf(dx*dx + dy*dy);
        remaining     -= traveled;

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

void Robot::turn(float deg) {
    if (fabsf(deg) < 0.5f) return;
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
    if (dist < 1.0f) goto arrival;

    {
        // 1. Tourne vers la cible
        float target_angle = atan2f(-dy, dx);   // Y+ vers le bas → dy inversé
        float delta = _normAngle(target_angle - _motion.getTheta());
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
}

// ─── Pose ─────────────────────────────────────────────────────────────────────

void Robot::setPosition(float x_mm, float y_mm, float theta_deg) {
    _motion.setPosition(x_mm, y_mm, theta_deg);
    ESP_LOGI(TAG, "Position: x=%.0f y=%.0f θ=%.1f°", x_mm, y_mm, theta_deg);
}

// ─── Cinématique ──────────────────────────────────────────────────────────────

void Robot::setSpeed(float mmS) {
    _motion.setSpeed(mmS);
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
