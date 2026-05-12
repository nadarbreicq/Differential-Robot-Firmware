#include "encoder.h"
#include "../log.h"

bool QuadEncoder::init(int pinA, int pinB, pcnt_unit_t unit) {
    _unit = unit;

    // Canal 0 : fronts sur A, direction par B
    pcnt_config_t cfgA = {};
    cfgA.pulse_gpio_num = pinA;
    cfgA.ctrl_gpio_num  = pinB;
    cfgA.pos_mode       = PCNT_COUNT_INC;     // front montant A → +1
    cfgA.neg_mode       = PCNT_COUNT_DEC;     // front descendant A → -1
    cfgA.hctrl_mode     = PCNT_MODE_REVERSE;  // B=1 → inverser sens
    cfgA.lctrl_mode     = PCNT_MODE_KEEP;     // B=0 → garder sens
    cfgA.counter_h_lim  =  32767;
    cfgA.counter_l_lim  = -32768;
    cfgA.unit           = unit;
    cfgA.channel        = PCNT_CHANNEL_0;

    // Canal 1 : fronts sur B, direction par A
    pcnt_config_t cfgB = {};
    cfgB.pulse_gpio_num = pinB;
    cfgB.ctrl_gpio_num  = pinA;
    cfgB.pos_mode       = PCNT_COUNT_DEC;     // front montant B → -1
    cfgB.neg_mode       = PCNT_COUNT_INC;     // front descendant B → +1
    cfgB.hctrl_mode     = PCNT_MODE_REVERSE;  // A=1 → inverser sens
    cfgB.lctrl_mode     = PCNT_MODE_KEEP;     // A=0 → garder sens
    cfgB.counter_h_lim  =  32767;
    cfgB.counter_l_lim  = -32768;
    cfgB.unit           = unit;
    cfgB.channel        = PCNT_CHANNEL_1;

    if (pcnt_unit_config(&cfgA) != ESP_OK) { LOG_E("ENC", "pcnt chan0 unit%d failed", unit); return false; }
    if (pcnt_unit_config(&cfgB) != ESP_OK) { LOG_E("ENC", "pcnt chan1 unit%d failed", unit); return false; }

    // Filtre anti-rebond : rejette impulsions < 1000 cycles APB (~12.5 ns × 1000 = 12.5 µs)
    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);

    _lastHw     = 0;
    _totalCount = 0;
    return true;
}

void QuadEncoder::reset() {
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);
    _lastHw     = 0;
    _totalCount = 0;
}

void QuadEncoder::update() {
    int16_t hw = 0;
    pcnt_get_counter_value(_unit, &hw);

    // Sur ESP32-S3, le PCNT se remet à 0 quand il atteint h_lim (32767) ou l_lim (-32768).
    // Il ne wrappe PAS en int16_t — il faut détecter le reset et recalculer le delta.
    int32_t delta;
    if (_lastHw > 16384 && hw < (_lastHw - 16384)) {
        // Reset depuis h_lim : compteur allait vers 32767, reset à 0, reparti à hw
        delta = (int32_t)(32767 - _lastHw) + (int32_t)hw;
    } else if (_lastHw < -16384 && hw > (_lastHw + 16384)) {
        // Reset depuis l_lim : compteur allait vers -32768, reset à 0, reparti à hw
        delta = (int32_t)(-32768 - _lastHw) + (int32_t)hw;
    } else {
        delta = (int32_t)hw - (int32_t)_lastHw;
    }

    _lastHw      = hw;
    _totalCount += delta;
}
