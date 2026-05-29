#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Commandes d'état envoyées depuis l'UI web → taskStrategy
enum class StateCmd : uint8_t {
    STOP_MOTORS,     // désactive moteurs immédiatement
    ENABLE_MOTORS,   // réactive les moteurs
    STOP_MATCH,      // force la fin du match (isMatchOver() → true)
    RESTART_INIT,    // relance runInitXxx depuis position actuelle
    RESTART_MATCH,   // relance la stratégie depuis position actuelle
    GOTO_WAIT_INIT,  // retour à l'état WAIT_INIT (boucle pré-match)
    START_MATCH,     // démarre le match (équivalent retrait tirette)
};

extern QueueHandle_t gStateCmd;
void sendStateCmd(StateCmd cmd);
