#include "pca9685.h"
#include <Arduino.h>

// ─── Registres PCA9685 ───────────────────────────────────────────────────────
static constexpr uint8_t REG_MODE1    = 0x00;
static constexpr uint8_t REG_PRESCALE = 0xFE;
static constexpr uint8_t REG_LED0_ON_L = 0x06;   // base canal 0

PCA9685::PCA9685(uint8_t addr, TwoWire &wire)
    : _addr(addr), _wire(wire), _freqHz(50.0f) {}

bool PCA9685::begin(float freqHz) {
    _freqHz = freqHz;

    writeReg(REG_MODE1, 0x00);   // reset
    delay(5);

    // Calcul prescaler : clk interne = 25 MHz, résolution = 4096
    float prescale = 25000000.0f / (4096.0f * freqHz) - 1.0f;
    uint8_t prescaleVal = (uint8_t)(prescale + 0.5f);

    uint8_t mode1 = readReg(REG_MODE1);
    writeReg(REG_MODE1, (mode1 & 0x7F) | 0x10);   // sleep (bit 4) pour écrire prescale
    writeReg(REG_PRESCALE, prescaleVal);
    writeReg(REG_MODE1, mode1);                     // réveiller
    delay(5);
    writeReg(REG_MODE1, mode1 | 0xA1);             // auto-increment + restart

    // Vérification présence
    _wire.beginTransmission(_addr);
    return _wire.endTransmission() == 0;
}

void PCA9685::setPulseUs(uint8_t channel, uint16_t us) {
    uint16_t off = (uint16_t)((float)us * _freqHz * 4096.0f / 1000000.0f + 0.5f);
    if (off > 4095) off = 4095;
    setPWM(channel, 0, off);
}

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t base = REG_LED0_ON_L + channel * 4;
    _wire.beginTransmission(_addr);
    _wire.write(base);
    _wire.write(on  & 0xFF);
    _wire.write(on  >> 8);
    _wire.write(off & 0xFF);
    _wire.write(off >> 8);
    _wire.endTransmission();
}

void PCA9685::off(uint8_t channel) {
    setPWM(channel, 0, 4096);   // full OFF bit
}

void PCA9685::writeReg(uint8_t reg, uint8_t val) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    _wire.endTransmission();
}

uint8_t PCA9685::readReg(uint8_t reg) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission(false);
    _wire.requestFrom(_addr, (uint8_t)1);
    return _wire.read();
}
