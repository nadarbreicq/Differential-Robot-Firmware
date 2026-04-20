#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "lidar/ld06.h"
#include "motion/step_control.h"
#include "strategy/robot.h"
#include "strategy/strategy.h"
#include "display/oled.h"

// ─── Instances globales ───────────────────────────────────────────────────────

static LD06         lidar(Serial1, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_PWM_PIN);
static StepControl  motion;
static Robot        robot(motion, lidar);

static LidarPoint   scanBuf[LD06_SCAN_BUF_SIZE];

// ─── Tâche LIDAR (Core 0) ────────────────────────────────────────────────────

static void taskLidar(void *) {
    lidar.begin();
    gDisplay.lidar_ok = true;
    for (;;) {
        lidar.update();
        vTaskDelay(1);
    }
}

// ─── Tâche Strategy (Core 1, prio 2) ─────────────────────────────────────────

static void taskStrategy(void *) {
    vTaskDelay(pdMS_TO_TICKS(2000));    // attend stabilisation LIDAR au démarrage
    gDisplay.robot_state = RobotState::IDLE;
    runStrategy(robot);
    gDisplay.robot_state = RobotState::DONE;
    ESP_LOGI("MAIN", "Stratégie terminée");
    vTaskDelete(nullptr);
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== DifferentialRobot ===");

    if (!motion.begin()) {
        Serial.println("[ERREUR] motion.begin() failed");
        while (true) vTaskDelay(1000);
    }

    displayStart();

    xTaskCreatePinnedToCore(taskLidar,    "lidar",    TASK_LIDAR_STACK,
                            nullptr, TASK_LIDAR_PRIO,    nullptr, TASK_LIDAR_CORE);

    xTaskCreatePinnedToCore(taskStrategy, "strategy", TASK_STRATEGY_STACK,
                            nullptr, TASK_STRATEGY_PRIO, nullptr, TASK_STRATEGY_CORE);
}

// ─── Loop : Teleplot + mise à jour affichage ─────────────────────────────────

void loop() {
    static uint32_t lastSend  = 0;
    static uint32_t lastStats = 0;
    uint32_t now = millis();

    // Mise à jour données écran
    gDisplay.lidar_rpm      = lidar.getRPM();
    gDisplay.pose_x_mm      = motion.getX();
    gDisplay.pose_y_mm      = motion.getY();
    gDisplay.pose_theta_deg = motion.getThetaDeg();

    // Envoi Teleplot LIDAR (1 pt / 2, toutes les 500 ms)
    if (now - lastSend >= 500) {
        lastSend = now;
        uint16_t n = lidar.getScan(scanBuf, LD06_SCAN_BUF_SIZE);
        gDisplay.lidar_pts = n;

        for (uint16_t i = 0; i < n; i += 2) {
            if (scanBuf[i].distance_mm == 0)   continue;
            if (scanBuf[i].confidence  < 100)  continue;
            if (scanBuf[i].distance_mm > 3000) continue;
            float rad = scanBuf[i].angle_deg * (3.14159265f / 180.0f);
            Serial.printf(">lidar:%.0f:%.0f|xy\n",
                          -scanBuf[i].distance_mm * cosf(rad),
                           scanBuf[i].distance_mm * sinf(rad));
        }

        // Pose robot sur Teleplot
        Serial.printf(">robot_x:%.0f\n", motion.getX());
        Serial.printf(">robot_y:%.0f\n", motion.getY());
        Serial.printf(">robot_theta:%.1f\n", motion.getThetaDeg());
    }

    if (now - lastStats >= 2000) {
        lastStats = now;
        Serial.printf("#LIDAR %.0f RPM | %u pts | x=%.0f y=%.0f θ=%.1f°\n",
                      lidar.getRPM(), gDisplay.lidar_pts,
                      motion.getX(), motion.getY(), motion.getThetaDeg());
    }

    vTaskDelay(10);
}
