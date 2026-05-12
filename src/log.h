#pragma once
#include <Arduino.h>
#include "config.h"

// ─── Niveaux : 0=off  1=error  2=warn  3=info  4=debug ───────────────────────
// Modifier LOG_LEVEL dans config.h.
// Le compilateur élimine les blocs inactifs (condition constante à compile-time).

#define LOG_E(tag, fmt, ...) do { if (LOG_LEVEL >= 1) Serial.printf("[E][" tag "] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_W(tag, fmt, ...) do { if (LOG_LEVEL >= 2) Serial.printf("[W][" tag "] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_I(tag, fmt, ...) do { if (LOG_LEVEL >= 3) Serial.printf("[I][" tag "] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_D(tag, fmt, ...) do { if (LOG_LEVEL >= 4) Serial.printf("[D][" tag "] " fmt "\n", ##__VA_ARGS__); } while(0)
