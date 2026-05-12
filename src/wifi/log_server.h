#pragma once
#include <Arduino.h>
#include "../config.h"

#if WIFI_LOG_ENABLED

#include <WebServer.h>
#include <WebSocketsServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "../lidar/ld06.h"

// ─── Structures ──────────────────────────────────────────────────────────────

struct LogEntry {
    char     level;
    char     tag[16];
    char     msg[96];
    uint32_t ts;
};

struct ServoCmd {
    char  id[24];   // "brasDroit" | "brasGauche" | "lifter" | "gripper" | "seq_*"
    float val;      // 0-100 (%)
};

// ─── LogServer ───────────────────────────────────────────────────────────────

class LogServer {
public:
    void begin();
    void push(char level, const char* tag, const char* msg);
    void updatePose(float x, float y, float theta_deg, const char* state,
                    float nav_dist_mm, float nav_delta_deg, char team);
    void updateLidar(const LidarPoint* buf, uint16_t n);
    void updateActuators(float bd, float bg, float li, float gr);
    bool pollCmd(ServoCmd& out);

private:
    static void _task(void* arg);
    void        _loop();
    void        _handleWsMsg(uint8_t* payload, size_t len);

    WebServer        _http{80};
    WebSocketsServer _ws{81};
    QueueHandle_t    _logQueue{nullptr};
    QueueHandle_t    _cmdQueue{nullptr};

    // Pose
    volatile float _px{0}, _py{0}, _ptheta{0};
    volatile float _navDist{0}, _navDelta{0};
    volatile bool  _poseNew{false};
    volatile char  _team{'?'};
    char           _pstate[16]{"?"};

    // LIDAR
    char          _lidarJson[5120];
    volatile bool _lidarNew{false};

    // Actionneurs
    volatile float _actBd{0}, _actBg{0}, _actLi{0}, _actGr{0};
    volatile bool  _actNew{false};
};

extern LogServer logServer;

// ─── Fonctions libres ────────────────────────────────────────────────────────
void wifiLogBegin();
void wifiLogPush(char level, const char* tag, const char* msg);
void wifiLogUpdatePose(float x, float y, float theta_deg, const char* state,
                       float nav_dist_mm, float nav_delta_deg, char team);
void wifiLogLidar(const LidarPoint* buf, uint16_t n);
void wifiLogUpdateActuators(float bd, float bg, float li, float gr);
bool wifiPollCmd(ServoCmd& out);

#else

inline void wifiLogBegin() {}
inline void wifiLogUpdatePose(float, float, float, const char*, float, float, char) {}
inline void wifiLogLidar(const void*, uint16_t) {}
inline void wifiLogUpdateActuators(float, float, float, float) {}
struct ServoCmd { char id[24]; float val; };
inline bool wifiPollCmd(ServoCmd&) { return false; }

#endif // WIFI_LOG_ENABLED
