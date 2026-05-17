#include "oled.h"
#include "../config.h"
#include <Wire.h>
#include <U8g2lib.h>
#include "esp_freertos_hooks.h"

DisplayData gDisplay = {};

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

// ─── Mesure charge CPU via idle hooks ────────────────────────────────────────
// Chaque core incrémente son compteur depuis sa tâche idle (priorité 0).
// Plus le CPU est libre, plus le compteur monte vite.

static volatile uint32_t s_idleCnt[2] = {0, 0};

static bool IRAM_ATTR idleHook0() { s_idleCnt[0]++; return false; }
static bool IRAM_ATTR idleHook1() { s_idleCnt[1]++; return false; }

static void updateCpuLoad() {
    static uint32_t prevCnt[2]  = {0, 0};
    static uint32_t baseline[2] = {1, 1};   // rolling max = 100 % idle

    for (int c = 0; c < 2; c++) {
        uint32_t cur   = s_idleCnt[c];
        uint32_t delta = cur - prevCnt[c];
        prevCnt[c]     = cur;

        // Le plus grand delta observé = référence 0 % de charge
        if (delta > baseline[c]) baseline[c] = delta;

        uint8_t load = (uint8_t)(100 - (delta * 100UL / baseline[c]));
        if (c == 0) gDisplay.cpu0_pct = load;
        else        gDisplay.cpu1_pct = load;
    }
}

// ─── Rendu ───────────────────────────────────────────────────────────────────

static void render() {
    char buf[32];
    u8g2.clearBuffer();

    RobotState s = gDisplay.robot_state;
    bool isYellow = (gDisplay.team == Team::YELLOW);

    // ── Helper : header centré fond inversé ─────────────────────────────────
    auto drawHeader = [&](const char *txt) {
        u8g2.setFont(u8g2_font_7x14B_tf);
        uint8_t sw = u8g2.getStrWidth(txt);
        u8g2.setDrawColor(1);
        u8g2.drawBox(0, 0, 128, 16);
        u8g2.setDrawColor(0);
        u8g2.drawStr((128 - sw) / 2, 13, txt);
        u8g2.setDrawColor(1);
    };

    // ── Helper : indicateur équipe en haut à droite dans le header ──────────
    auto drawTeamBadge = [&]() {
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.setDrawColor(0);
        u8g2.drawStr(121, 13, isYellow ? "J" : "B");
        u8g2.setDrawColor(1);
    };

    // ════════════════════════════════════════════════════════════════════════
    // PRÉ-MATCH
    // ════════════════════════════════════════════════════════════════════════
    if (s == RobotState::WAIT_INIT || s == RobotState::INIT
     || s == RobotState::WAIT_TIRETTE_IN || s == RobotState::WAIT_TIRETTE_OUT) {

        drawHeader(robotStateStr(s));

        // Équipe en grand — couleur différente selon l'équipe
        u8g2.setFont(u8g2_font_10x20_tf);
        const char *teamStr = isYellow ? "JAUNE" : "BLEU";
        uint8_t tw = u8g2.getStrWidth(teamStr);
        u8g2.drawStr((128 - tw) / 2, 38, teamStr);

        // IP WiFi (ou LIDAR seul si pas de WiFi)
        u8g2.setFont(u8g2_font_6x10_tf);
        if (gDisplay.wifi_ip[0]) {
            snprintf(buf, sizeof(buf), "IP: %s", gDisplay.wifi_ip);
            u8g2.drawStr(0, 51, buf);
        }
        snprintf(buf, sizeof(buf), "LIDAR:%s  R:%.0f  L:%.0f",
                 gDisplay.lidar_ok ? "OK" : "--",
                 (double)(gDisplay.enc_right_cnt * MM_PER_COUNT),
                 (double)(gDisplay.enc_left_cnt  * MM_PER_COUNT));
        u8g2.drawStr(0, 63, buf);

    // ════════════════════════════════════════════════════════════════════════
    // OBSTACLE
    // ════════════════════════════════════════════════════════════════════════
    } else if (s == RobotState::OBSTACLE) {

        drawHeader("OBSTACLE!");
        drawTeamBadge();

        // Distance en grand
        u8g2.setFont(u8g2_font_10x20_tf);
        snprintf(buf, sizeof(buf), "%.0f mm", (double)gDisplay.obs_dist_mm);
        uint8_t dw = u8g2.getStrWidth(buf);
        u8g2.drawStr((128 - dw) / 2, 38, buf);

        // Direction
        float a = gDisplay.obs_angle_deg;
        const char *sector = (fabsf(a) < 30.0f) ? "AVANT"
                           : (fabsf(a) > 150.0f) ? "ARRIERE"
                           : (a > 0)             ? "AV.GAUCHE"
                                                 : "AV.DROITE";
        u8g2.setFont(u8g2_font_7x14B_tf);
        uint8_t sw2 = u8g2.getStrWidth(sector);
        u8g2.drawStr((128 - sw2) / 2, 55, sector);

        u8g2.setFont(u8g2_font_6x10_tf);
        snprintf(buf, sizeof(buf), "%+.0fdeg  L:%s",
                 (double)gDisplay.obs_angle_deg,
                 gDisplay.lidar_ok ? "OK" : "--");
        u8g2.drawStr(0, 63, buf);

    // ════════════════════════════════════════════════════════════════════════
    // MATCH (MOVING, GOTO, TURNING, IDLE, ENDGAME, DONE)
    // ════════════════════════════════════════════════════════════════════════
    } else {

        drawHeader(robotStateStr(s));
        drawTeamBadge();

        // Position encodeur (principale référence)
        u8g2.setFont(u8g2_font_7x14B_tf);
        snprintf(buf, sizeof(buf), "X:%.0f  Y:%.0f",
                 (double)gDisplay.enc_pose_x_mm,
                 (double)gDisplay.enc_pose_y_mm);
        u8g2.drawStr(0, 30, buf);

        snprintf(buf, sizeof(buf), "%.1f",
                 (double)gDisplay.enc_pose_theta_deg);
        u8g2.drawStr(0, 44, buf);

        // Navigation en cours
        u8g2.setFont(u8g2_font_6x10_tf);
        if (gDisplay.nav_dist_mm > 0.5f) {
            snprintf(buf, sizeof(buf), "D:%.0fmm  d:%+.1f",
                     (double)gDisplay.nav_dist_mm,
                     (double)gDisplay.nav_delta_deg);
            u8g2.drawStr(0, 55, buf);
        }

        // Temps restant + LIDAR
        uint32_t elapsed = (gDisplay.match_start_ms > 0)
                         ? (millis() - gDisplay.match_start_ms) / 1000 : 0;
        uint32_t remain  = (elapsed < 100) ? (100 - elapsed) : 0;
        snprintf(buf, sizeof(buf), "%lus rest.  L:%s",
                 remain, gDisplay.lidar_ok ? "OK" : "--");
        u8g2.drawStr(0, 63, buf);
    }

    u8g2.sendBuffer();
}

// ─── Tâche ───────────────────────────────────────────────────────────────────

static void taskDisplay(void *) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        updateCpuLoad();
        render();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

// ─── Init ────────────────────────────────────────────────────────────────────

void displayStart() {
    esp_register_freertos_idle_hook_for_cpu(idleHook0, 0);
    esp_register_freertos_idle_hook_for_cpu(idleHook1, 1);

    if (!u8g2.begin()) return;   // écran absent : on ne lance pas la tâche
    u8g2.setContrast(128);

    xTaskCreatePinnedToCore(taskDisplay, "display", TASK_DISPLAY_STACK, nullptr,
                            TASK_DISPLAY_PRIO, nullptr, TASK_DISPLAY_CORE);
}
