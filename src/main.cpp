#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "config.h"
#include "lidar/ld06.h"
#include "motion/step_control.h"
#include "strategy/robot.h"
#include "strategy/strategy.h"
#include "display/oled.h"
#include "io/buttons.h"
#include "io/leds.h"
#include "actuators/actuators.h"
#include "motion/encoder.h"

// ─── Instances globales ───────────────────────────────────────────────────────

static LD06         lidar(Serial1, LIDAR_RX_PIN, -1, LIDAR_PWM_PIN);
static StepControl  motion;
static QuadEncoder  encRight;
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
    // ── Phase pré-match ───────────────────────────────────────────────────────
    robot.disableMotors();   // moteurs libres pendant le positionnement
    encRight.init(ENC_RIGHT_A_PIN, ENC_RIGHT_B_PIN, PCNT_UNIT_2);  // après FastAccelStepper
    gDisplay.team = teamSwitch();
    ledsSetTeam(gDisplay.team);
    gDisplay.robot_state = RobotState::WAIT_INIT;

    // États : WAIT_INIT → WAIT_TIRETTE_IN → WAIT_TIRETTE_OUT → match
    enum class Phase : uint8_t { WAIT_INIT, WAIT_TIRETTE_IN, WAIT_TIRETTE_OUT };
    Phase phase = Phase::WAIT_INIT;

    // Condition de sortie : tirette retirée après init + mise en place
    while (phase != Phase::WAIT_TIRETTE_OUT || !tirette()) {

        // Switch d'équipe lu en continu ; changement = init à refaire
        Team t = teamSwitch();
        if (t != gDisplay.team) {
            gDisplay.team = t;
            ledsSetTeam(t);
            phase = Phase::WAIT_INIT;
            gDisplay.robot_state = RobotState::WAIT_INIT;
        }

        switch (phase) {
        case Phase::WAIT_INIT:
            if (initPressed()) {
                gDisplay.robot_state = RobotState::INIT;
                if (gDisplay.team == Team::YELLOW) runInitYellow(robot);
                else                               runInitBlue(robot);
                phase = Phase::WAIT_TIRETTE_IN;
                gDisplay.robot_state = RobotState::WAIT_TIRETTE_IN;
            }
            break;

        case Phase::WAIT_TIRETTE_IN:
            if (!tirette()) {
                phase = Phase::WAIT_TIRETTE_OUT;
                gDisplay.robot_state = RobotState::WAIT_TIRETTE_OUT;
            }
            break;

        case Phase::WAIT_TIRETTE_OUT:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // ── Match ─────────────────────────────────────────────────────────────────
    robot.startMatch();

    if (gDisplay.team == Team::YELLOW) runStrategyYellow(robot);
    else                               runStrategyBlue(robot);

    // ── Repli fin de match (si on est dans la fenêtre 80-100 s) ───────────────
    if (robot.isEndgame() && !robot.isMatchOver()) {
        gDisplay.robot_state = RobotState::ENDGAME;
        if (gDisplay.team == Team::YELLOW) runNearEndYellow(robot);
        else                               runNearEndBlue(robot);
    }

    // ── Attente fin de match puis désengagement ────────────────────────────────
    while (!robot.isMatchOver()) vTaskDelay(pdMS_TO_TICKS(50));
    robot.disableMotors();
    actuatorsDisable();
    gDisplay.robot_state = RobotState::DONE;
    ESP_LOGI("MAIN", "Match terminé");
    vTaskDelete(nullptr);
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== DifferentialRobot ===");

    Wire.begin(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);
    Wire.setClock(400000);

    if (!motion.begin()) {
        Serial.println("[ERREUR] motion.begin() failed");
        while (true) vTaskDelay(1000);
    }

    buttonsInit();
    ledsInit();
    actuatorsInit();

    displayStart();

    xTaskCreatePinnedToCore(taskLidar,    "lidar",    TASK_LIDAR_STACK,
                            nullptr, TASK_LIDAR_PRIO,    nullptr, TASK_LIDAR_CORE);

    xTaskCreatePinnedToCore(taskStrategy, "strategy", TASK_STRATEGY_STACK,
                            nullptr, TASK_STRATEGY_PRIO, nullptr, TASK_STRATEGY_CORE);
}

// ─── Loop : LEDs + Teleplot + affichage ──────────────────────────────────────

void loop() {
    static uint32_t lastLed       = 0;
    static uint32_t lastLidarPlot = 0;
    static uint32_t lastPosePlot  = 0;
    static uint32_t lastStats     = 0;
    uint32_t now = millis();

    // Mise à jour données écran
    gDisplay.lidar_rpm      = lidar.getRPM();
    gDisplay.pose_x_mm      = motion.getX();
    gDisplay.pose_y_mm      = motion.getY();
    gDisplay.pose_theta_deg = motion.getThetaDeg();

    // Encodeur droit — update 100 Hz (gère le rollover hardware)
    encRight.update();
    if (gDisplay.robot_state == RobotState::WAIT_INIT) {
        gDisplay.enc_right_cnt = encRight.getCount();
    }

    // LEDs LIDAR — toujours actif (100 ms)
    if (now - lastLed >= 100) {
        lastLed = now;
        uint16_t n = lidar.getScan(scanBuf, LD06_SCAN_BUF_SIZE);
        gDisplay.lidar_pts = n;
        ledsUpdateLidar(scanBuf, n);
    }

    // Teleplot LIDAR + encodeur — uniquement en phase WAIT_INIT (200 ms)
    if (gDisplay.robot_state == RobotState::WAIT_INIT &&
        now - lastLidarPlot >= 200) {
        lastLidarPlot = now;
        uint16_t n = lidar.getScan(scanBuf, LD06_SCAN_BUF_SIZE);
        for (uint16_t i = 0; i < n; i += 2) {
            if (scanBuf[i].distance_mm == 0)   continue;
            if (scanBuf[i].confidence  < 100)  continue;
            if (scanBuf[i].distance_mm > 3000) continue;
            float rad = scanBuf[i].angle_deg * (3.14159265f / 180.0f);
            Serial.printf(">lidar:%.0f:%.0f|xy\n",
                          -scanBuf[i].distance_mm * cosf(rad),
                           scanBuf[i].distance_mm * sinf(rad));
        }
        Serial.printf(">enc_R:%ld\n", (long)encRight.getCount());
        Serial.printf(">enc_R_mm:%.2f\n", encRight.getCount() * MM_PER_COUNT);
    }

    // Teleplot pose robot — pendant le match (500 ms)
    if (gDisplay.robot_state != RobotState::WAIT_INIT &&
        gDisplay.robot_state != RobotState::WAIT_TIRETTE_IN &&
        gDisplay.robot_state != RobotState::WAIT_TIRETTE_OUT &&
        now - lastPosePlot >= 500) {
        lastPosePlot = now;
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
