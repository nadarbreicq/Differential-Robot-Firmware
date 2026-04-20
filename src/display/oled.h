#pragma once
#include <stdint.h>

enum class RobotState : uint8_t {
    IDLE,
    MOVING,
    TURNING,
    GOTO,
    OBSTACLE,
    DONE,
    ERROR
};

static inline const char* robotStateStr(RobotState s) {
    switch (s) {
        case RobotState::IDLE:     return "PRET";
        case RobotState::MOVING:   return "AVANCE";
        case RobotState::TURNING:  return "TOURNE";
        case RobotState::GOTO:     return "GOTO";
        case RobotState::OBSTACLE: return "OBSTACLE!";
        case RobotState::DONE:     return "FIN";
        case RobotState::ERROR:    return "ERREUR";
        default:                   return "?";
    }
}

// Données partagées entre les tâches et l'écran.
// Écriture atomique (types 32 bits alignés sur ESP32).
struct DisplayData {
    volatile float      pose_x_mm;
    volatile float      pose_y_mm;
    volatile float      pose_theta_deg;
    volatile RobotState robot_state;
    volatile float      lidar_rpm;
    volatile uint16_t   lidar_pts;
    volatile bool       lidar_ok;
    volatile uint8_t    cpu0_pct;
    volatile uint8_t    cpu1_pct;
};

extern DisplayData gDisplay;

void displayStart();
