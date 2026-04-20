#include "oled.h"
#include "../config.h"
#include <Wire.h>
#include <U8g2lib.h>
#include "esp_freertos_hooks.h"

DisplayData gDisplay = {};

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

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

    // ── Ligne 1 : état robot (grande police, centré) ──────────────────────────
    u8g2.setFont(u8g2_font_7x14B_tf);
    const char *stateStr = robotStateStr(gDisplay.robot_state);
    uint8_t sw = u8g2.getStrWidth(stateStr);
    // Fond inversé pour mise en évidence
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 16);
    u8g2.setDrawColor(0);
    u8g2.drawStr((128 - sw) / 2, 13, stateStr);
    u8g2.setDrawColor(1);

    // ── Ligne 2 : position X / Y ─────────────────────────────────────────────
    u8g2.setFont(u8g2_font_6x10_tf);
    snprintf(buf, sizeof(buf), "X:%7.1f mm", (double)gDisplay.pose_x_mm);
    u8g2.drawStr(0, 27, buf);
    snprintf(buf, sizeof(buf), "Y:%7.1f mm", (double)gDisplay.pose_y_mm);
    u8g2.drawStr(0, 38, buf);

    // ── Ligne 3 : cap ────────────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "Cap: %6.1f deg", (double)gDisplay.pose_theta_deg);
    u8g2.drawStr(0, 49, buf);

    // ── Ligne 4 : LIDAR + CPU ────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%s %4.0fRPM C0:%2u C1:%2u%%",
             gDisplay.lidar_ok ? "L:OK" : "L:--",
             (double)gDisplay.lidar_rpm,
             gDisplay.cpu0_pct, gDisplay.cpu1_pct);
    u8g2.drawStr(0, 61, buf);

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

    Wire.begin(DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);
    Wire.setClock(400000);

    if (!u8g2.begin()) return;   // écran absent : on ne lance pas la tâche
    u8g2.setContrast(128);

    xTaskCreatePinnedToCore(taskDisplay, "display", TASK_DISPLAY_STACK, nullptr,
                            TASK_DISPLAY_PRIO, nullptr, TASK_DISPLAY_CORE);
}
