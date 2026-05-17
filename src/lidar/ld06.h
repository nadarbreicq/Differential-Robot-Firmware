#pragma once
#include <Arduino.h>

#define LD06_PACKET_LEN     47
#define LD06_HEADER         0x54
#define LD06_VER_LEN        0x2C
#define LD06_PTS_PER_PKT    12
#define LD06_SCAN_BUF_SIZE  500

struct LidarPoint {
    float    angle_deg;   // [0, 360)
    uint16_t distance_mm;
    uint8_t  confidence;
};

class LD06 {
public:
    LD06(HardwareSerial &serial, uint8_t rxPin, uint8_t pwmPin);  // LD06 one-way RX only

    void begin();
    void update();   // à appeler en boucle dans la tâche LIDAR

    // Copie thread-safe du dernier scan complet (360°)
    uint16_t getScan(LidarPoint *dst, uint16_t maxPts);

    float getRPM() const { return _rpm; }

private:
    HardwareSerial &_serial;
    uint8_t _rxPin, _pwmPin;

    // Réception paquet
    uint8_t  _raw[LD06_PACKET_LEN];
    uint8_t  _idx;
    bool     _synced;

    // Buffer scan (accumule une révolution complète)
    LidarPoint _pending[LD06_SCAN_BUF_SIZE];
    uint16_t   _pendingCnt;
    float      _lastAngle;

    // Buffer publié (accès thread-safe)
    LidarPoint      _scan[LD06_SCAN_BUF_SIZE];
    uint16_t        _scanCnt;
    portMUX_TYPE    _mux;

    float _rpm;

    void processByte(uint8_t b);
    bool parsePacket();
    static uint8_t crc8(const uint8_t *data, uint8_t len);
};
