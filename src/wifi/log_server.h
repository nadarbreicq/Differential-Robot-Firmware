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
                    float nav_dist_mm, float nav_delta_deg, char team, bool canClick);
    void updateLidar(const LidarPoint* buf, uint16_t n);
    void updateLidarAbs(const LidarPoint* buf, uint16_t n,
                        float robot_x, float robot_y, float robot_theta_rad);
    void updateActuators(float bd, float bg, float li, float gr);
    // Motion debug : état/phase + cible courante + vitesses commandées.
    // state/phase codés en uint8_t (cf. MotionController::State/Phase).
    void updateMotion(uint8_t state, uint8_t phase,
                      float distMm, float speedCap,
                      float vL, float vR,
                      float tgtX, float tgtY, float tgtTheta);
    bool pollCmd(ServoCmd& out);
    bool pollFieldClick(float& x_mm, float& y_mm, float& theta_deg);

private:
    static void _task(void* arg);
    void        _loop();
    void        _handleWsMsg(uint8_t* payload, size_t len);
    void        _handleCalibMsg(const char* p);
    void        _buildCalibJson();

    WebServer        _http{80};
    WebSocketsServer _ws{81};
    QueueHandle_t    _logQueue{nullptr};
    QueueHandle_t    _cmdQueue{nullptr};

    // Pose
    volatile float _px{0}, _py{0}, _ptheta{0};
    volatile float _navDist{0}, _navDelta{0};
    volatile bool  _poseNew{false};
    volatile bool  _canClick{false};
    volatile char  _team{'?'};
    char           _pstate[16]{"?"};

    // LIDAR radar (relatif robot)
    char          _lidarJson[5120];
    volatile bool _lidarNew{false};

    // LIDAR absolu (coordonnées table)
    char          _lidarAbsJson[5120];
    volatile bool _lidarAbsNew{false};

    // Actionneurs
    volatile float _actBd{0}, _actBg{0}, _actLi{0}, _actGr{0};
    volatile bool  _actNew{false};

    // Motion debug
    volatile uint8_t _motSt{0}, _motPh{0};
    volatile float   _motDist{0}, _motCap{0}, _motVL{0}, _motVR{0};
    volatile float   _motTgtX{0}, _motTgtY{0}, _motTgtTheta{0};
    volatile bool    _motNew{false};

    // Field click (utilisateur clique sur le terrain dans l'UI).
    // _fcTheta = NAN si l'utilisateur n'a pas drag (pas de contrainte orientation).
    volatile float _fcX{0}, _fcY{0}, _fcTheta{0};
    volatile bool  _fcHasTheta{false};
    volatile bool  _fcPending{false};

    // Calibration
    char          _calibJson[512];
    volatile bool _calibNew{false};
};

extern LogServer logServer;

// ─── Fonctions libres ────────────────────────────────────────────────────────
void wifiLogBegin();
void wifiLogPush(char level, const char* tag, const char* msg);
void wifiLogUpdatePose(float x, float y, float theta_deg, const char* state,
                       float nav_dist_mm, float nav_delta_deg, char team, bool canClick);
void wifiLogLidar(const LidarPoint* buf, uint16_t n);
void wifiLogLidarAbs(const LidarPoint* buf, uint16_t n,
                     float robot_x, float robot_y, float robot_theta_rad);
void wifiLogUpdateActuators(float bd, float bg, float li, float gr);
void wifiLogUpdateMotion(uint8_t state, uint8_t phase,
                         float distMm, float speedCap,
                         float vL, float vR,
                         float tgtX, float tgtY, float tgtTheta);
bool wifiPollCmd(ServoCmd& out);

// Click utilisateur sur le terrain. Renvoie true si une commande était en
// attente (et remet le flag à false). x/y en mm coordonnées table.
// theta_deg = NAN si pas de contrainte orientation (click sans drag).
bool wifiPollFieldClick(float& x_mm, float& y_mm, float& theta_deg);

#else

inline void wifiLogBegin() {}
inline void wifiLogUpdatePose(float, float, float, const char*, float, float, char, bool) {}
inline void wifiLogLidar(const void*, uint16_t) {}
inline void wifiLogUpdateActuators(float, float, float, float) {}
inline void wifiLogUpdateMotion(uint8_t, uint8_t, float, float, float, float, float, float, float) {}
struct ServoCmd { char id[24]; float val; };
inline bool wifiPollCmd(ServoCmd&) { return false; }
inline bool wifiPollFieldClick(float&, float&, float&) { return false; }

#endif // WIFI_LOG_ENABLED
