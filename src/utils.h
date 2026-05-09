#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Equivalent de delay() mais libère le CPU pour les autres tâches FreeRTOS
inline void wait(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
