#pragma once
#include <stdint.h>
#include <Wire.h>

// Driver PCF8574 — expandeur GPIO 8 bits I2C.
// Chaque bit peut être sortie (LOW=actif) ou entrée (écrire HIGH puis lire).
class PCF8574 {
public:
    PCF8574(uint8_t addr = 0x20, TwoWire &wire = Wire);

    bool begin(uint8_t initialState = 0xFF);  // 0xFF = tous en entrée/repos

    // Écriture / lecture d'un pin individuel (0-7)
    void     setPin(uint8_t pin, bool high);
    bool     getPin(uint8_t pin);

    // Accès direct au byte complet
    void    writeByte(uint8_t val);
    uint8_t readByte();

private:
    uint8_t  _addr;
    TwoWire &_wire;
    uint8_t  _state;   // état de sortie courant
};
