#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "config.h"
#include "live_config.h"
#include "log.h"
#include "wifi/log_server.h"
#include "lidar/ld06.h"
#include "motion/step_control.h"
#include "motion/motion_ctrl.h"
#include "strategy/robot.h"
#include "strategy/strategy.h"
#include "display/oled.h"
#include "io/buttons.h"
#include "io/leds.h"
#include "actuators/actuators.h"
#include "motion/encoder.h"

// ─── Instances globales ───────────────────────────────────────────────────────

static LD06         lidar(Serial1, LIDAR_RX_PIN, LIDAR_PWM_PIN);
static StepControl  motion;
static QuadEncoder  encRight;
static QuadEncoder  encLeft;
static Robot        robot(motion, lidar, gMotionCtrl);

static LidarPoint   scanBuf[LD06_SCAN_BUF_SIZE];

// ─── Tâche encodeurs (Core 0, 200 Hz) ───────────────────────────────────────
// Tâche dédiée pour gérer le rollover PCNT (±32767) indépendamment du reste.

static void taskEncoders(void *) {
    static int32_t lastL   = 0;
    static int32_t lastR   = 0;
    static float   encTheta = 0.0f;
    static constexpr float PI_F = 3.14159265f;

    for (;;) {
        encRight.update();
        encLeft.update();

        int32_t curL = encLeft.getCount();
        int32_t curR = encRight.getCount();

        // Comptes bruts toujours disponibles (affichage WAIT_INIT)
        gDisplay.enc_left_cnt  = curL;
        gDisplay.enc_right_cnt = curR;

        // Reset odométrie si setPosition() l'a demandé
        if (gDisplay.enc_reset_pending) {
            gDisplay.enc_reset_pending  = false;
            gDisplay.enc_pose_x_mm      = gDisplay.enc_reset_x;
            gDisplay.enc_pose_y_mm      = gDisplay.enc_reset_y;
            gDisplay.enc_pose_theta_deg = gDisplay.enc_reset_theta_deg;
            encTheta = gDisplay.enc_reset_theta_deg * (PI_F / 180.0f);
            lastL = curL;
            lastR = curR;
        }

        // Odométrie différentielle encodeurs
        int32_t dL = curL - lastL;
        int32_t dR = curR - lastR;
        lastL = curL;
        lastR = curR;

        float leftMm  = (float)dL * gCalib.mmPerCount;
        float rightMm = (float)dR * gCalib.mmPerCount;
        float dDist   = (leftMm + rightMm) * 0.5f;
        float dTheta  = (rightMm - leftMm) / gCalib.encWheelbase;
        float mid     = encTheta + dTheta * 0.5f;

        gDisplay.enc_pose_x_mm += dDist * cosf(mid);
        gDisplay.enc_pose_y_mm -= dDist * sinf(mid);  // Y+ vers le bas
        encTheta += dTheta;
        while (encTheta >  PI_F) encTheta -= 2.0f * PI_F;
        while (encTheta < -PI_F) encTheta += 2.0f * PI_F;
        gDisplay.enc_pose_theta_rad = encTheta;
        gDisplay.enc_pose_theta_deg = encTheta * (180.0f / PI_F);

        vTaskDelay(pdMS_TO_TICKS(5));   // 200 Hz
    }
}

// ─── Tâche LIDAR (Core 0) ────────────────────────────────────────────────────

static void taskLidar(void *) {
    lidar.begin();
    gDisplay.lidar_ok = true;
    for (;;) {
        lidar.update();
        vTaskDelay(1);
    }
}

// ─── Commandes UI → Strategy ─────────────────────────────────────────────────

#include "state_cmd.h"

QueueHandle_t gStateCmd = nullptr;

void sendStateCmd(StateCmd cmd) {
    if (gStateCmd) xQueueSend(gStateCmd, &cmd, 0);
}

// ─── Tâche Strategy (Core 1, prio 2) ─────────────────────────────────────────

static void taskStrategy(void *) {
    // Encodeurs initialisés une seule fois (PCNT ne supporte pas le re-init)
    encRight.init(ENC_RIGHT_INVERT ? ENC_RIGHT_B_PIN : ENC_RIGHT_A_PIN,
                  ENC_RIGHT_INVERT ? ENC_RIGHT_A_PIN : ENC_RIGHT_B_PIN, PCNT_UNIT_2);
    encLeft.init( ENC_LEFT_INVERT  ? ENC_LEFT_B_PIN  : ENC_LEFT_A_PIN,
                  ENC_LEFT_INVERT  ? ENC_LEFT_A_PIN  : ENC_LEFT_B_PIN,  PCNT_UNIT_3);
    robot.setEncoders(&encLeft, &encRight);

    // taskMotionControl : 50 Hz sur Core 1, prio 3 — démarre une fois que les
    // encodeurs sont prêts (la tâche les lit dès le premier tick).
    gMotionCtrl.begin(&motion, &encLeft, &encRight, &lidar);

restart:   // point de retour pour GOTO_WAIT_INIT / RESTART_MATCH
    robot.disableMotors();
    gDisplay.team = teamSwitch();
    ledsSetTeam(gDisplay.team);
    gDisplay.robot_state = RobotState::WAIT_INIT;

    // États : WAIT_INIT → WAIT_TIRETTE_IN → WAIT_TIRETTE_OUT → match
    enum class Phase : uint8_t { WAIT_INIT, WAIT_TIRETTE_IN, WAIT_TIRETTE_OUT };
    Phase phase = Phase::WAIT_INIT;

    // Condition de sortie : tirette retirée après init + mise en place
    while (phase != Phase::WAIT_TIRETTE_OUT || !tirette()) {

        // Commandes UI
        StateCmd cmd;
        if (xQueueReceive(gStateCmd, &cmd, 0) == pdTRUE) {
            if (cmd == StateCmd::STOP_MOTORS)    { robot.disableMotors(); }
            if (cmd == StateCmd::ENABLE_MOTORS)  { robot.enableMotors(); }
            if (cmd == StateCmd::GOTO_WAIT_INIT) goto restart;
            if (cmd == StateCmd::START_MATCH)    goto match_start;   // skip tirette
            if (cmd == StateCmd::RESTART_INIT)  {
                gDisplay.robot_state = RobotState::INIT;
                if (gDisplay.team == Team::YELLOW) runInitYellow(robot);
                else                               runInitBlue(robot);
                phase = Phase::WAIT_TIRETTE_IN;
                gDisplay.robot_state = RobotState::WAIT_TIRETTE_IN;
            }
        }

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
match_start:
    robot.startMatch();

    if (gDisplay.team == Team::YELLOW) runStrategyYellow(robot);
    else                               runStrategyBlue(robot);

    // ── Attente endgame (vérifie aussi RESTART_MATCH) ────────────────────────
    while (!robot.isEndgame() && !robot.isMatchOver()) {
        StateCmd cmd;
        if (xQueueReceive(gStateCmd, &cmd, 0) == pdTRUE) {
            if (cmd == StateCmd::STOP_MOTORS)    { robot.disableMotors(); }
            if (cmd == StateCmd::ENABLE_MOTORS)  { robot.enableMotors(); }
            if (cmd == StateCmd::STOP_MATCH)     { gDisplay.match_start_ms = millis() - MATCH_DURATION_MS; }
            if (cmd == StateCmd::RESTART_MATCH)  { robot.disableMotors(); goto restart; }
            if (cmd == StateCmd::GOTO_WAIT_INIT) { robot.disableMotors(); goto restart; }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ── Repli fin de match ────────────────────────────────────────────────────
    if (!robot.isMatchOver()) {
        robot.startNearEnd();
        gDisplay.robot_state = RobotState::ENDGAME;
        if (gDisplay.team == Team::YELLOW) runNearEndYellow(robot);
        else                               runNearEndBlue(robot);
    }

    while (!robot.isMatchOver()) vTaskDelay(pdMS_TO_TICKS(50));
    robot.disableMotors();
    actuatorsDisable();
    gDisplay.robot_state = RobotState::DONE;
    LOG_I("MAIN", "Match termine");

    // ── Après match : attente commande UI pour relancer ───────────────────────
    while (true) {
        StateCmd cmd;
        if (xQueueReceive(gStateCmd, &cmd, 0) == pdTRUE) {
            if (cmd == StateCmd::STOP_MOTORS)    { robot.disableMotors(); }
            if (cmd == StateCmd::ENABLE_MOTORS)  { robot.enableMotors(); }
            if (cmd == StateCmd::GOTO_WAIT_INIT) { goto restart; }
            if (cmd == StateCmd::RESTART_MATCH ||
                cmd == StateCmd::START_MATCH)    { robot.enableMotors(); goto match_start; }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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

    gStateCmd = xQueueCreate(4, sizeof(StateCmd));
    buttonsInit();
    ledsInit();
    actuatorsInit();
    wifiLogBegin();

    displayStart();

    xTaskCreatePinnedToCore(taskLidar,    "lidar",    TASK_LIDAR_STACK,
                            nullptr, TASK_LIDAR_PRIO,    nullptr, TASK_LIDAR_CORE);

    xTaskCreatePinnedToCore(taskEncoders, "encoders", 2048,
                            nullptr, 3,                  nullptr, 0);   // Core 0, prio 3

    xTaskCreatePinnedToCore(taskStrategy, "strategy", TASK_STRATEGY_STACK,
                            nullptr, TASK_STRATEGY_PRIO, nullptr, TASK_STRATEGY_CORE);
}

// ─── Loop : LEDs + WiFi telemetry + affichage ────────────────────────────────

void loop() {
    static uint32_t lastLed      = 0;
    static uint32_t lastMatchLog = 0;
    static uint32_t lastWifi     = 0;
    uint32_t now = millis();

    // Mise à jour données écran
    gDisplay.lidar_rpm      = lidar.getRPM();
    gDisplay.pose_x_mm      = motion.getX();
    gDisplay.pose_y_mm      = motion.getY();
    gDisplay.pose_theta_deg = motion.getThetaDeg();

    // Log match — 500 ms pendant le match
    if (LOG_LEVEL >= 3 && gDisplay.match_start_ms > 0 && now - lastMatchLog >= 500) {
        lastMatchLog = now;
        uint32_t elapsed = (now - gDisplay.match_start_ms) / 1000;
        Serial.printf("[%3lus] %s | Est(%5.0f,%5.0f,%+5.1f) Enc(%5.0f,%5.0f,%+5.1f)\n",
                      elapsed,
                      robotStateStr(gDisplay.robot_state),
                      (double)gDisplay.pose_x_mm,      (double)gDisplay.pose_y_mm,
                      (double)gDisplay.pose_theta_deg,
                      (double)gDisplay.enc_pose_x_mm,  (double)gDisplay.enc_pose_y_mm,
                      (double)gDisplay.enc_pose_theta_deg);
    }

    // LEDs + LIDAR WiFi — 100 ms (LIDAR WiFi seulement en pré-match)
    if (now - lastLed >= 100) {
        lastLed = now;
        uint16_t n = lidar.getScan(scanBuf, LD06_SCAN_BUF_SIZE);
        gDisplay.lidar_pts = n;
        ledsUpdateLidar(scanBuf, n);
        RobotState st = gDisplay.robot_state;
        if (st == RobotState::WAIT_INIT      || st == RobotState::INIT ||
            st == RobotState::WAIT_TIRETTE_IN || st == RobotState::WAIT_TIRETTE_OUT)
            wifiLogLidar(scanBuf, n);
        // LIDAR absolu toujours envoyé (coordonnées table, filtré en JS)
        wifiLogLidarAbs(scanBuf, n,
                        gDisplay.enc_pose_x_mm, gDisplay.enc_pose_y_mm,
                        gDisplay.enc_pose_theta_rad);
    }

    // Pose + actionneurs + motion debug + commandes WiFi — 200 ms
    if (now - lastWifi >= 200) {
        lastWifi = now;
        char team = (gDisplay.team == Team::YELLOW) ? 'Y' : 'B';
        // canClick : autorisé après init et hors match en cours
        RobotState rs = gDisplay.robot_state;
        bool postInit  = (rs != RobotState::WAIT_INIT) && (rs != RobotState::INIT);
        bool inMatch   = (gDisplay.match_start_ms > 0) &&
                         (now - gDisplay.match_start_ms < MATCH_DURATION_MS);
        bool canClick  = postInit && !inMatch;
        wifiLogUpdatePose(gDisplay.enc_pose_x_mm, gDisplay.enc_pose_y_mm, gDisplay.enc_pose_theta_deg,
                          robotStateStr(gDisplay.robot_state),
                          gDisplay.nav_dist_mm, gDisplay.nav_delta_deg, team, canClick);
        wifiLogUpdateActuators(servoBrasDroit.getPercent(),
                               servoBrasGauche.getPercent(),
                               servoLifter.getPercent(),
                               servoGripper.getPercent());
        wifiLogUpdateMotion((uint8_t)gMotionCtrl.getState(),
                            (uint8_t)gMotionCtrl.getPhase(),
                            gMotionCtrl.getDistMm(),
                            gMotionCtrl.getSpeedCap(),
                            gMotionCtrl.getCmdL(),
                            gMotionCtrl.getCmdR(),
                            gMotionCtrl.getTargetX(),
                            gMotionCtrl.getTargetY(),
                            gMotionCtrl.getTargetTheta());
    }

    // Click utilisateur sur le terrain → goto vers (x, y[, theta]).
    // Autorisé après init (state != WAIT_INIT/INIT) et hors match en cours
    // (match_start_ms == 0 OU match déjà fini).
    float fcX, fcY, fcTheta;
    if (wifiPollFieldClick(fcX, fcY, fcTheta)) {
        RobotState st = gDisplay.robot_state;
        bool postInit = (st != RobotState::WAIT_INIT) && (st != RobotState::INIT);
        bool inMatch  = (gDisplay.match_start_ms > 0) &&
                        (millis() - gDisplay.match_start_ms < MATCH_DURATION_MS);
        if (postInit && !inMatch) {
            robot.enableMotors();
            robot.setTarget(fcX, fcY, fcTheta);   // fcTheta = NAN si pas de drag
            if (isnanf(fcTheta)) LOG_I("UI", "Goto click: %.0f,%.0f", (double)fcX, (double)fcY);
            else                 LOG_I("UI", "Goto click: %.0f,%.0f theta=%.0f", (double)fcX, (double)fcY, (double)fcTheta);
        } else {
            LOG_W("UI", "Goto ignore (etat %d, inMatch=%d)", (int)st, (int)inMatch);
        }
    }

    // Commandes actionneurs depuis l'interface web (seulement avant init)
    ServoCmd cmd;
    while (wifiPollCmd(cmd)) {
        if (gDisplay.robot_state != RobotState::WAIT_INIT) continue;

        if      (strcmp(cmd.id, "seq_init")  == 0) initActuators();
        else if (strcmp(cmd.id, "seq_prise") == 0) sequencePrise();
        else if (strcmp(cmd.id, "brasDroit")   == 0) servoBrasDroit.setPercent(cmd.val);
        else if (strcmp(cmd.id, "brasGauche")  == 0) servoBrasGauche.setPercent(cmd.val);
        else if (strcmp(cmd.id, "lifter")      == 0) servoLifter.setPercent(cmd.val);
        else if (strcmp(cmd.id, "gripper")     == 0) servoGripper.setPercent(cmd.val);
    }

    vTaskDelay(10);
}
