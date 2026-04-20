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

    gDisplay.robot_state = RobotState::MOVING;
    float dir = _motion.getTheta() + (mm < 0 ? PI_F : 0.0f);

    _motion.startGo(mm);

    while (_motion.isMoving()) {
        vTaskDelay(pdMS_TO_TICKS(OBS_POLL_MS));

        if (_obstacleEn && _obstacleInDir(dir)) {
            _motion.stop();
            _motion.syncPose();
            gDisplay.robot_state = RobotState::OBSTACLE;
            ESP_LOGW(TAG, "Obstacle détecté — arrêt");
            _waitObstacleClear(dir);
            gDisplay.robot_state = RobotState::MOVING;
            // Recalcule la distance restante et reprend
            float dx = _motion.getX();
            float dy = _motion.getY();
            (void)dx; (void)dy;
            // TODO : reprendre le mouvement avec la distance restante
            // Pour l'instant on s'arrête définitivement
            return;
        }
    }
    _motion.syncPose();
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
        float target_angle = atan2f(dy, dx);
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
    uint16_t n = _lidar.getScan(_scanBuf, LD06_SCAN_BUF_SIZE);
    if (n == 0) return false;

    float rx = _motion.getX();
    float ry = _motion.getY();
    float rTheta = _motion.getTheta();

    for (uint16_t i = 0; i < n; i++) {
        if (_scanBuf[i].distance_mm == 0) continue;
        if (_scanBuf[i].confidence < OBS_CONFIDENCE_MIN) continue;

        float d = _scanBuf[i].distance_mm;

        // Point en repère robot (LIDAR aligné avec l'avant du robot)
        float lidarAngle = (_scanBuf[i].angle_deg + LIDAR_OFFSET_DEG) * DEG2RAD;
        float px = d * cosf(lidarAngle);   // avant positif
        float py = d * sinf(lidarAngle);   // gauche positif

        // Coordonnées monde pour filtrage table
        float wx, wy;
        _lidarToWorld(_scanBuf[i].angle_deg, d, wx, wy);

        // Filtre murs : si le point est hors table → c'est un mur, on ignore
        if (wx < TABLE_MARGIN_MM || wx > TABLE_WIDTH_MM  - TABLE_MARGIN_MM) continue;
        if (wy < TABLE_MARGIN_MM || wy > TABLE_HEIGHT_MM - TABLE_MARGIN_MM) continue;

        // Zone de détection : rectangle dans le sens du mouvement
        // On projette (px, py) dans le repère "direction de mouvement"
        float delta = dir_rad - rTheta;   // angle du mouvement par rapport au robot
        float fwd  =  px * cosf(delta) + py * sinf(delta);
        float lat  = -px * sinf(delta) + py * cosf(delta);

        if (fwd > OBS_MIN_DIST_MM && fwd < OBS_DETECT_DIST_MM &&
            fabsf(lat) < OBS_WIDTH_MM * 0.5f) {
            return true;
        }
    }
    (void)rx; (void)ry; (void)rTheta;
    return false;
}

void Robot::_lidarToWorld(float angle_deg, float dist_mm,
                          float &wx, float &wy) const {
    float a   = (angle_deg + LIDAR_OFFSET_DEG) * DEG2RAD + _motion.getTheta();
    wx = _motion.getX() + dist_mm * cosf(a);
    wy = _motion.getY() + dist_mm * sinf(a);
}

void Robot::_waitObstacleClear(float dir_rad) {
    uint32_t start = millis();
    while (_obstacleInDir(dir_rad)) {
        if (millis() - start > OBS_WAIT_MS) {
            ESP_LOGW(TAG, "Timeout obstacle — action subsidiaire à implémenter");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Obstacle dégagé");
}

float Robot::_normAngle(float rad) {
    while (rad >  PI_F) rad -= 2.0f * PI_F;
    while (rad < -PI_F) rad += 2.0f * PI_F;
    return rad;
}
