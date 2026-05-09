#include "leds.h"
#include "../config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t s_teamColor = 0;

void ledsInit() {
    strip.begin();
    strip.setBrightness(80);
    strip.show();
}

void ledsSetTeam(Team t) {
    s_teamColor = (t == Team::YELLOW)
        ? strip.Color(255, 160,   0)
        : strip.Color(  0,   0, 200);
    strip.setPixelColor(0, s_teamColor);
    strip.show();
}

// ─── LIDAR debug ─────────────────────────────────────────────────────────────
//
// Layout physique (d'après schéma robot) :
//          AVANT
//   [4]  [5]  [6]
//        [0]          ← équipe
//   [3]  [2]  [1]
//        ARRIÈRE

struct SectorDef { uint8_t led; float start; float end; };

static const SectorDef kSectors[6] = {
    {5, 330.0f,  30.0f},   // avant-centre   (passage par 0°)
    {4,  30.0f,  90.0f},   // avant-gauche
    {3,  90.0f, 150.0f},   // arrière-gauche
    {2, 150.0f, 210.0f},   // arrière-centre
    {1, 210.0f, 270.0f},   // arrière-droite
    {6, 270.0f, 330.0f},   // avant-droite
};

static float normalizeAngle(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static bool inSector(float angle, float start, float end) {
    if (start < end) return angle >= start && angle < end;
    return angle >= start || angle < end;   // secteur à cheval sur 0°
}

void ledsUpdateLidar(const LidarPoint *pts, uint16_t n) {
    bool hit[6] = {};

    for (uint16_t i = 0; i < n; i++) {
        const LidarPoint &p = pts[i];
        if (p.distance_mm == 0 || p.confidence < OBS_CONFIDENCE_MIN) continue;
        float dist = (float)p.distance_mm;
        if (dist < LIDAR_BODY_DIST_MM || dist > LIDAR_LED_DIST_MM)     continue;

        float angle = normalizeAngle(270.0f - p.angle_deg + LIDAR_OFFSET_DEG);

        // Zones aveugles (poteaux structurels)
        if ((angle >= LIDAR_BLIND_L_START && angle <= LIDAR_BLIND_L_END) ||
            (angle >= LIDAR_BLIND_R_START && angle <= LIDAR_BLIND_R_END)) continue;

        for (uint8_t s = 0; s < 6; s++) {
            if (inSector(angle, kSectors[s].start, kSectors[s].end)) {
                hit[s] = true;
                break;
            }
        }
    }

    strip.setPixelColor(0, s_teamColor);
    for (uint8_t s = 0; s < 6; s++) {
        strip.setPixelColor(kSectors[s].led,
            hit[s] ? strip.Color(200,   0,  0)   // rouge  = obstacle
                   : strip.Color(  0,  20,  0));  // vert dim = libre
    }
    strip.show();
}
