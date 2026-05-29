#pragma once
#include "config.h"

// Paramètres de calibration modifiables en RAM via l'interface web.
// Initialisés depuis config.h — les valeurs d'origine restent dans config.h.
// Thread safety : écritures isolées et rares (calibration seulement).
// Les float 32-bit sur ESP32 sont alignés → lecture/écriture atomiques.
struct Calib {
    // ── Mécanique ────────────────────────────────────────────────────────────
    float encWheelDiamMm   = ENC_WHEEL_DIAM_MM;
    float encWheelbase     = ENC_WHEELBASE_MM;
    float wheelbase        = WHEELBASE_MM;
    float driveWheelDiamMm = DRIVE_WHEEL_DIAM_MM;
    float lidarOffsetDeg   = LIDAR_OFFSET_DEG;

    // Dérivés — recalculés quand le diamètre change
    float mmPerCount = 3.14159265f * ENC_WHEEL_DIAM_MM / (ENC_PPR * 4.0f);
    float stepsPerMm = (float)STEPPER_STEPS_REV / (3.14159265f * DRIVE_WHEEL_DIAM_MM);

    // ── PID ──────────────────────────────────────────────────────────────────
    float kp           = ENC_P1_KP;
    float ki           = ENC_P1_KI;
    float kd           = ENC_P1_KD;
    float iMax         = ENC_P1_I_MAX;
    float stopMm       = ENC_P1_STOP_MM;
    float minSpd       = ENC_P1_MIN_SPD;
    float defaultSpeed = DEFAULT_SPEED_MMS;
    float defaultAccel = DEFAULT_ACCEL_MMS2;
};

extern Calib gCalib;
