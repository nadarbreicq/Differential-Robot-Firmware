#pragma once
#include <Arduino.h>
#include "config.h"

// ─── Niveaux : 0=off  1=error  2=warn  3=info  4=debug ───────────────────────
// Modifier LOG_LEVEL dans config.h.
// Le compilateur élimine les blocs inactifs (condition constante à compile-time).

// ─── Interface WiFi log (implémentée dans wifi/log_server.cpp) ───────────────
#if WIFI_LOG_ENABLED
void wifiLogBegin();
void wifiLogPush(char level, const char* tag, const char* msg);
void wifiLogUpdatePose(float x, float y, float theta_deg, const char* state);
#define _LOG_WIFI(lvl, tag, fmt, ...) \
    do { char _wlb[96]; snprintf(_wlb, sizeof(_wlb), fmt, ##__VA_ARGS__); \
         wifiLogPush(lvl, tag, _wlb); } while(0)
#else
inline void wifiLogBegin() {}
inline void wifiLogUpdatePose(float, float, float, const char*) {}
#define _LOG_WIFI(lvl, tag, fmt, ...)
#endif

// ─── Macros LOG ───────────────────────────────────────────────────────────────
#define LOG_E(tag, fmt, ...) do { if (LOG_LEVEL >= 1) { Serial.printf("[E][" tag "] " fmt "\n", ##__VA_ARGS__); _LOG_WIFI('E', tag, fmt, ##__VA_ARGS__); } } while(0)
#define LOG_W(tag, fmt, ...) do { if (LOG_LEVEL >= 2) { Serial.printf("[W][" tag "] " fmt "\n", ##__VA_ARGS__); _LOG_WIFI('W', tag, fmt, ##__VA_ARGS__); } } while(0)
#define LOG_I(tag, fmt, ...) do { if (LOG_LEVEL >= 3) { Serial.printf("[I][" tag "] " fmt "\n", ##__VA_ARGS__); _LOG_WIFI('I', tag, fmt, ##__VA_ARGS__); } } while(0)
#define LOG_D(tag, fmt, ...) do { if (LOG_LEVEL >= 4) { Serial.printf("[D][" tag "] " fmt "\n", ##__VA_ARGS__); _LOG_WIFI('D', tag, fmt, ##__VA_ARGS__); } } while(0)
