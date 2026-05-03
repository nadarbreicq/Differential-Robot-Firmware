#include "pcf8574.h"

PCF8574::PCF8574(uint8_t addr, TwoWire &wire)
    : _addr(addr), _wire(wire), _state(0xFF) {}

bool PCF8574::begin(uint8_t initialState) {
    _state = initialState;
    writeByte(_state);
    _wire.beginTransmission(_addr);
    return _wire.endTransmission() == 0;
}

void PCF8574::setPin(uint8_t pin, bool high) {
    if (pin > 7) return;
    if (high) _state |=  (1 << pin);
    else      _state &= ~(1 << pin);
    writeByte(_state);
}

bool PCF8574::getPin(uint8_t pin) {
    if (pin > 7) return false;
    // Pour lire, le bit correspondant doit être HIGH côté sortie
    if (!(_state & (1 << pin))) {
        _state |= (1 << pin);
        writeByte(_state);
    }
    return (readByte() >> pin) & 0x01;
}

void PCF8574::writeByte(uint8_t val) {
    _state = val;
    _wire.beginTransmission(_addr);
    _wire.write(val);
    _wire.endTransmission();
}

uint8_t PCF8574::readByte() {
    _wire.requestFrom(_addr, (uint8_t)1);
    if (_wire.available()) return _wire.read();
    return 0xFF;
}
