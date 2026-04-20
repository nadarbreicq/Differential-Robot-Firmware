#pragma once
#include <stdint.h>
#include "driver/pcnt.h"   // ESP-IDF 4.x PCNT API

// Décodeur quadrature x4 utilisant le PCNT hardware ESP32-S3.
// Accumule sur 32 bits en compensant le rollover ±32767 hardware.
class QuadEncoder {
public:
    QuadEncoder() = default;

    // unit : PCNT_UNIT_0 … PCNT_UNIT_3
    bool init(int pinA, int pinB, pcnt_unit_t unit);

    void reset();
    void update();   // appeler ≥ 100 Hz

    int32_t getCount() const { return _totalCount; }

private:
    pcnt_unit_t _unit    = PCNT_UNIT_0;
    int16_t     _lastHw  = 0;
    int32_t     _totalCount = 0;
};
