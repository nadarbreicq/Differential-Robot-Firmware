#pragma once
#include <stdint.h>
#include "../config.h"

enum class RobotState : uint8_t {
    WAIT_INIT,           // pré-match : attente appui bouton init
    INIT,                // calage bordure en cours
    WAIT_TIRETTE_IN,     // attente insertion tirette
    WAIT_TIRETTE_OUT,    // tirette en place, attente retrait → départ
    IDLE,                // entre deux mouvements
    MOVING,
    TURNING,
    GOTO,
    OBSTACLE,
    ENDGAME,            // repli fin de match en cours
    DONE,
    ERROR
};

static inline const char* robotStateStr(RobotState s) {
    switch (s) {
        case RobotState::WAIT_INIT:       return "APPUYER INIT";
        case RobotState::INIT:            return "CALAGE...";
        case RobotState::WAIT_TIRETTE_IN: return "INSERER TIRETTE";
        case RobotState::WAIT_TIRETTE_OUT:return "PRET !";
        case RobotState::IDLE:            return "PRET";
        case RobotState::MOVING:          return "AVANCE";
        case RobotState::TURNING:         return "TOURNE";
        case RobotState::GOTO:            return "GOTO";
        case RobotState::OBSTACLE:        return "OBSTACLE!";
        case RobotState::ENDGAME:         return "REPLI !";
        case RobotState::DONE:            return "MATCH FINI";
        case RobotState::ERROR:           return "ERREUR";
        default:                          return "?";
    }
}

// Données partagées entre les tâches et l'écran.
// Écriture atomique (types 32 bits alignés sur ESP32).
struct DisplayData {
    volatile float      pose_x_mm;
    volatile float      pose_y_mm;
    volatile float      pose_theta_deg;
    volatile RobotState robot_state;
    volatile Team       team;
    volatile float      lidar_rpm;
    volatile uint16_t   lidar_pts;
    volatile bool       lidar_ok;
    volatile uint8_t    cpu0_pct;
    volatile uint8_t    cpu1_pct;
    // Obstacle détecté (mis à jour par robot.cpp)
    volatile float      obs_dist_mm;
    volatile float      obs_angle_deg;  // repère robot : 0°=avant, +90°=gauche
    volatile uint32_t   match_start_ms; // millis() au départ — 0 = pas démarré
};

extern DisplayData gDisplay;

void displayStart();
