#pragma once
#include <stdint.h>
#include <Wire.h>

// Driver minimaliste PCA9685 — 16 canaux PWM I2C pour servomoteurs.
// Fréquence par défaut 50 Hz (standard servo).
class PCA9685 {
public:
    PCA9685(uint8_t addr = 0x40, TwoWire &wire = Wire);

    bool begin(float freqHz = 50.0f);

    // Pulse en microsecondes (ex : 1000-2000 µs pour servo standard)
    void setPulseUs(uint8_t channel, uint16_t us);

    // Contrôle brut : valeurs ON/OFF dans [0, 4095]
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);

    // Éteint un canal
    void off(uint8_t channel);

private:
    uint8_t   _addr;
    TwoWire  &_wire;
    float     _freqHz;

    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
};
