#include "log_server.h"
#include <math.h>

#if WIFI_LOG_ENABLED

#include <WiFi.h>
#include <LittleFS.h>
#include "../credentials.h"
#include "../config.h"
#include "../live_config.h"
#include "../display/oled.h"

LogServer logServer;

// ─── Fonctions libres ─────────────────────────────────────────────────────────

void wifiLogBegin()  { logServer.begin(); }

void wifiLogPush(char level, const char* tag, const char* msg) {
    logServer.push(level, tag, msg);
}

void wifiLogUpdatePose(float x, float y, float theta_deg, const char* state,
                       float nav_dist_mm, float nav_delta_deg, char team, bool canClick) {
    logServer.updatePose(x, y, theta_deg, state, nav_dist_mm, nav_delta_deg, team, canClick);
}

void wifiLogLidar(const LidarPoint* buf, uint16_t n) {
    logServer.updateLidar(buf, n);
}

void wifiLogLidarAbs(const LidarPoint* buf, uint16_t n,
                     float robot_x, float robot_y, float robot_theta_rad) {
    logServer.updateLidarAbs(buf, n, robot_x, robot_y, robot_theta_rad);
}

void wifiLogUpdateActuators(float bd, float bg, float li, float gr) {
    logServer.updateActuators(bd, bg, li, gr);
}

void wifiLogUpdateMotion(uint8_t state, uint8_t phase,
                         float distMm, float speedCap,
                         float vL, float vR,
                         float tgtX, float tgtY, float tgtTheta) {
    logServer.updateMotion(state, phase, distMm, speedCap, vL, vR, tgtX, tgtY, tgtTheta);
}

bool wifiPollFieldClick(float& x_mm, float& y_mm, float& theta_deg) {
    return logServer.pollFieldClick(x_mm, y_mm, theta_deg);
}

bool LogServer::pollFieldClick(float& x_mm, float& y_mm, float& theta_deg) {
    if (!_fcPending) return false;
    x_mm = _fcX;
    y_mm = _fcY;
    theta_deg = _fcHasTheta ? _fcTheta : NAN;
    _fcPending = false;
    return true;
}

bool wifiPollCmd(ServoCmd& out) {
    return logServer.pollCmd(out);
}

// ─── begin ────────────────────────────────────────────────────────────────────

void LogServer::begin() {
    _logQueue = xQueueCreate(64, sizeof(LogEntry));
    _cmdQueue = xQueueCreate(8,  sizeof(ServoCmd));

    WiFi.setTxPower(WIFI_POWER_19_5dBm);

#if WIFI_USE_STA
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    Serial.printf("[I][WIFI] Connexion à %s ...\n", WIFI_STA_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000)
        vTaskDelay(pdMS_TO_TICKS(500));
    if (WiFi.status() == WL_CONNECTED) {
        String ip = WiFi.localIP().toString();
        Serial.printf("[I][WIFI] Connecté — http://%s\n", ip.c_str());
        strncpy(gDisplay.wifi_ip, ip.c_str(), sizeof(gDisplay.wifi_ip) - 1);
    } else {
        Serial.println("[W][WIFI] STA échoué — repli en mode AP");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
        Serial.printf("[I][WIFI] AP: %s  IP: %s\n", WIFI_AP_SSID,
                      WiFi.softAPIP().toString().c_str());
        strncpy(gDisplay.wifi_ip, "AP:" WIFI_AP_SSID, sizeof(gDisplay.wifi_ip) - 1);
    }
#else
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
    Serial.printf("[I][WIFI] AP: %s  ch=%d  IP: %s\n", WIFI_AP_SSID, WIFI_AP_CHANNEL,
                  WiFi.softAPIP().toString().c_str());
    strncpy(gDisplay.wifi_ip, "AP:" WIFI_AP_SSID, sizeof(gDisplay.wifi_ip) - 1);
#endif

    if (!LittleFS.begin(false))
        Serial.println("[E][WIFI] LittleFS mount failed — lance 'pio run -t uploadfs'");

    _http.on("/", [this]() {
        File f = LittleFS.open("/index.html", "r");
        if (!f) { _http.send(404, "text/plain", "Not found"); return; }
        _http.streamFile(f, "text/html; charset=utf-8");
        f.close();
    });
    _http.on("/field.jpg", [this]() {
        File f = LittleFS.open("/field.jpg", "r");
        if (!f) { _http.send(404, "text/plain", "Not found"); return; }
        _http.streamFile(f, "image/jpeg");
        f.close();
    });
    _http.on("/poi.js", [this]() {
        File f = LittleFS.open("/poi.js", "r");
        if (!f) { _http.send(404, "text/plain", "Not found"); return; }
        _http.streamFile(f, "application/javascript; charset=utf-8");
        f.close();
    });
    _http.onNotFound([this]() { _http.send(404, "text/plain", "Not found"); });
    _http.begin();

    _ws.onEvent([this](uint8_t, WStype_t type, uint8_t* payload, size_t len) {
        if (type == WStype_CONNECTED) { _buildCalibJson(); _calibNew = true; }
        else if (type == WStype_TEXT && len > 4) _handleWsMsg(payload, len);
    });
    _ws.begin();

    xTaskCreatePinnedToCore(_task, "logwifi", TASK_WIFI_STACK, this,
                            TASK_WIFI_PRIO, nullptr, TASK_WIFI_CORE);
}

// ─── Réception WebSocket → commandes actionneurs ──────────────────────────────

#include "../state_cmd.h"

void LogServer::_handleWsMsg(uint8_t* payload, size_t len) {
    const char* p = (const char*)payload;

    // ── Calibration live ─────────────────────────────────────────────────────
    if (strstr(p, "\"calib\"")) { _handleCalibMsg(p); return; }

    // ── Click utilisateur sur le terrain ──────────────────────────────────────
    // Format : {"type":"goto","x":1234,"y":567}        (sans orientation)
    //        : {"type":"goto","x":1234,"y":567,"t":90}  (avec orientation finale)
    if (strstr(p, "\"goto\"")) {
        const char* xp = strstr(p, "\"x\":");
        const char* yp = strstr(p, "\"y\":");
        const char* tp = strstr(p, "\"t\":");
        if (xp && yp) {
            _fcX = (float)atof(xp + 4);
            _fcY = (float)atof(yp + 4);
            if (tp) {
                _fcTheta    = (float)atof(tp + 4);
                _fcHasTheta = true;
            } else {
                _fcHasTheta = false;
            }
            _fcPending = true;
        }
        return;
    }

    // ── Commandes état robot ──────────────────────────────────────────────────
    if (strstr(p, "\"state_cmd\"")) {
        // Ordre important : "start_match" doit être checké avant "stop_match"
        // car ils partagent un préfixe différent — pas de conflit ici, mais
        // "restart_match" partage avec "start_match" si on n'est pas précis.
        if      (strstr(p, "stop_motors"))    sendStateCmd(StateCmd::STOP_MOTORS);
        else if (strstr(p, "enable_motors"))  sendStateCmd(StateCmd::ENABLE_MOTORS);
        else if (strstr(p, "stop_match"))     sendStateCmd(StateCmd::STOP_MATCH);
        else if (strstr(p, "restart_init"))   sendStateCmd(StateCmd::RESTART_INIT);
        else if (strstr(p, "restart_match"))  sendStateCmd(StateCmd::RESTART_MATCH);
        else if (strstr(p, "start_match"))    sendStateCmd(StateCmd::START_MATCH);
        else if (strstr(p, "wait_init"))      sendStateCmd(StateCmd::GOTO_WAIT_INIT);
        return;
    }

    // ── Commandes actionneurs ─────────────────────────────────────────────────
    // Format attendu : {"type":"servo","id":"brasDroit","val":65.0}
    if (!strstr(p, "\"servo\"")) return;

    ServoCmd cmd = {};

    const char* idPos = strstr(p, "\"id\":\"");
    if (idPos) {
        idPos += 6;
        const char* end = strchr(idPos, '"');
        if (end) {
            int l = (int)(end - idPos);
            if (l > (int)sizeof(cmd.id) - 1) l = sizeof(cmd.id) - 1;
            memcpy(cmd.id, idPos, l);
        }
    }
    const char* valPos = strstr(p, "\"val\":");
    if (valPos) cmd.val = (float)atof(valPos + 6);

    if (cmd.id[0]) xQueueSend(_cmdQueue, &cmd, 0);
}

// ─── push ────────────────────────────────────────────────────────────────────

void LogServer::push(char level, const char* tag, const char* msg) {
    if (!_logQueue) return;
    LogEntry e;
    e.level = level;
    e.ts    = millis();
    strncpy(e.tag, tag, sizeof(e.tag) - 1);  e.tag[sizeof(e.tag) - 1] = '\0';
    strncpy(e.msg, msg, sizeof(e.msg) - 1);  e.msg[sizeof(e.msg) - 1] = '\0';
    for (char* c = e.msg; *c; ++c) if (*c == '"') *c = '\'';
    xQueueSend(_logQueue, &e, 0);
}

// ─── updatePose ───────────────────────────────────────────────────────────────

void LogServer::updatePose(float x, float y, float theta_deg, const char* state,
                           float nav_dist_mm, float nav_delta_deg, char team, bool canClick) {
    _px = x; _py = y; _ptheta = theta_deg;
    _navDist = nav_dist_mm; _navDelta = nav_delta_deg;
    _team = team;
    _canClick = canClick;
    strncpy(_pstate, state, sizeof(_pstate) - 1);
    _pstate[sizeof(_pstate) - 1] = '\0';
    _poseNew = true;
}

// ─── updateLidar ─────────────────────────────────────────────────────────────

void LogServer::updateLidar(const LidarPoint* buf, uint16_t n) {
    char local[5120];
    int  pos = 0;
    pos += snprintf(local + pos, sizeof(local) - pos, "{\"type\":\"lidar\",\"pts\":[");
    bool first = true;
    for (uint16_t i = 0; i < n; i += 2) {
        if (buf[i].confidence  < OBS_CONFIDENCE_MIN)                continue;
        if (buf[i].distance_mm < (uint16_t)LIDAR_BODY_DIST_MM)     continue;
        if (buf[i].distance_mm > 3000)                              continue;
        if (pos >= (int)sizeof(local) - 20)                         break;
        pos += snprintf(local + pos, sizeof(local) - pos, "%s[%d,%d]",
                        first ? "" : ",",
                        (int)buf[i].angle_deg, (int)buf[i].distance_mm);
        first = false;
    }
    pos += snprintf(local + pos, sizeof(local) - pos, "]}");
    memcpy(_lidarJson, local, pos + 1);
    _lidarNew = true;
}

// ─── updateLidarAbs ───────────────────────────────────────────────────────────

void LogServer::updateLidarAbs(const LidarPoint* buf, uint16_t n,
                                float robot_x, float robot_y, float robot_theta_rad) {
    // Projette chaque point LIDAR en coordonnées table (mm)
    // Même transformation que Robot::_lidarToWorld()
    // wx = robot_x + d * cos((270° - angle + offset) + theta_robot)
    // wy = robot_y - d * sin(...)   (Y+ vers le bas)
    static constexpr float DEG2RAD_F = 3.14159265f / 180.0f;

    char local[5120];
    int  pos = 0;
    pos += snprintf(local + pos, sizeof(local) - pos, "{\"type\":\"lidar_abs\",\"pts\":[");
    bool first = true;
    for (uint16_t i = 0; i < n; i += 3) {   // sous-échantillonnage x3
        if (buf[i].confidence  < OBS_CONFIDENCE_MIN)            continue;
        if (buf[i].distance_mm < (uint16_t)LIDAR_BODY_DIST_MM) continue;
        if (buf[i].distance_mm > 3000)                          continue;
        if (pos >= (int)sizeof(local) - 24)                     break;

        float a  = (270.0f - buf[i].angle_deg + gCalib.lidarOffsetDeg) * DEG2RAD_F
                   + robot_theta_rad;
        float d  = (float)buf[i].distance_mm;
        float wx = robot_x + d * cosf(a);
        float wy = robot_y - d * sinf(a);

        // Filtre : uniquement les points dans les limites de la table
        if (wx < 0 || wx > TABLE_WIDTH_MM)  continue;
        if (wy < 0 || wy > TABLE_HEIGHT_MM) continue;

        pos += snprintf(local + pos, sizeof(local) - pos, "%s[%d,%d]",
                        first ? "" : ",", (int)wx, (int)wy);
        first = false;
    }
    pos += snprintf(local + pos, sizeof(local) - pos, "]}");
    memcpy(_lidarAbsJson, local, pos + 1);
    _lidarAbsNew = true;
}

// ─── updateActuators ──────────────────────────────────────────────────────────

void LogServer::updateActuators(float bd, float bg, float li, float gr) {
    _actBd = bd; _actBg = bg; _actLi = li; _actGr = gr;
    _actNew = true;
}

void LogServer::updateMotion(uint8_t state, uint8_t phase,
                              float distMm, float speedCap,
                              float vL, float vR,
                              float tgtX, float tgtY, float tgtTheta) {
    _motSt = state; _motPh = phase;
    _motDist = distMm; _motCap = speedCap;
    _motVL = vL; _motVR = vR;
    _motTgtX = tgtX; _motTgtY = tgtY; _motTgtTheta = tgtTheta;
    _motNew = true;
}

// ─── pollCmd ─────────────────────────────────────────────────────────────────

bool LogServer::pollCmd(ServoCmd& out) {
    return xQueueReceive(_cmdQueue, &out, 0) == pdTRUE;
}

// ─── Tâche WiFi ───────────────────────────────────────────────────────────────

void LogServer::_task(void* arg) { static_cast<LogServer*>(arg)->_loop(); }

void LogServer::_loop() {
    for (;;) {
        _ws.loop();
        _http.handleClient();

        // Logs
        LogEntry e;
        while (xQueueReceive(_logQueue, &e, 0) == pdTRUE) {
            char buf[140];
            snprintf(buf, sizeof(buf),
                     "{\"l\":\"%c\",\"t\":\"%s\",\"m\":\"%s\",\"ts\":%lu}",
                     e.level, e.tag, e.msg, (unsigned long)e.ts);
            _ws.broadcastTXT(buf);
        }

        // Pose
        if (_poseNew) {
            _poseNew = false;
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "{\"type\":\"pose\",\"x\":%.1f,\"y\":%.1f,\"a\":%.1f,"
                     "\"state\":\"%s\",\"nd\":%.0f,\"na\":%.1f,\"tm\":\"%c\",\"cc\":%d}",
                     (double)_px, (double)_py, (double)_ptheta,
                     _pstate, (double)_navDist, (double)_navDelta, (char)_team,
                     _canClick ? 1 : 0);
            _ws.broadcastTXT(buf);
        }

        // LIDAR radar (relatif)
        if (_lidarNew) {
            _lidarNew = false;
            _ws.broadcastTXT(_lidarJson);
        }

        // LIDAR absolu (coordonnées table)
        if (_lidarAbsNew) {
            _lidarAbsNew = false;
            _ws.broadcastTXT(_lidarAbsJson);
        }

        // Actionneurs
        if (_actNew) {
            _actNew = false;
            char buf[96];
            snprintf(buf, sizeof(buf),
                     "{\"type\":\"act\",\"bd\":%.1f,\"bg\":%.1f,\"li\":%.1f,\"gr\":%.1f}",
                     (double)_actBd, (double)_actBg, (double)_actLi, (double)_actGr);
            _ws.broadcastTXT(buf);
        }

        // Calibration state
        if (_calibNew) {
            _calibNew = false;
            _ws.broadcastTXT(_calibJson);
        }

        // Motion debug + cible
        if (_motNew) {
            _motNew = false;
            char buf[256];
            // Note : tgtTheta peut être NaN → on envoie -9999 comme sentinelle
            float tT = isnanf((float)_motTgtTheta) ? -9999.0f : (float)_motTgtTheta;
            snprintf(buf, sizeof(buf),
                "{\"type\":\"motion\",\"st\":%u,\"ph\":%u,"
                "\"dist\":%.0f,\"cap\":%.0f,\"vL\":%.0f,\"vR\":%.0f,"
                "\"tx\":%.0f,\"ty\":%.0f,\"tt\":%.1f}",
                (unsigned)_motSt, (unsigned)_motPh,
                (double)_motDist, (double)_motCap,
                (double)_motVL, (double)_motVR,
                (double)_motTgtX, (double)_motTgtY, (double)tT);
            _ws.broadcastTXT(buf);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Calibration ─────────────────────────────────────────────────────────────

void LogServer::_buildCalibJson() {
    snprintf(_calibJson, sizeof(_calibJson),
        "{\"type\":\"calib_state\","
        "\"encWheelDiam\":%.3f,\"encWheelbase\":%.2f,\"wheelbase\":%.2f,"
        "\"driveWheelDiam\":%.3f,\"lidarOffset\":%.2f,"
        "\"kp\":%.4f,\"ki\":%.5f,\"kd\":%.4f,\"iMax\":%.1f,"
        "\"stopMm\":%.2f,\"minSpd\":%.2f,"
        "\"defaultSpeed\":%.0f,\"defaultAccel\":%.0f}",
        (double)gCalib.encWheelDiamMm,   (double)gCalib.encWheelbase,
        (double)gCalib.wheelbase,         (double)gCalib.driveWheelDiamMm,
        (double)gCalib.lidarOffsetDeg,
        (double)gCalib.kp,  (double)gCalib.ki,  (double)gCalib.kd, (double)gCalib.iMax,
        (double)gCalib.stopMm,  (double)gCalib.minSpd,
        (double)gCalib.defaultSpeed, (double)gCalib.defaultAccel);
}

void LogServer::_handleCalibMsg(const char* p) {
    static constexpr float PI_F = 3.14159265f;

    char key[32] = {};
    const char* kp = strstr(p, "\"key\":\"");
    if (kp) {
        kp += 7;
        const char* ke = strchr(kp, '"');
        if (ke) { int l = (int)(ke - kp); if (l > 0 && l < 32) memcpy(key, kp, l); }
    }
    const char* vp = strstr(p, "\"val\":");
    if (!vp || !key[0]) return;
    float val = (float)atof(vp + 6);

    if      (strcmp(key, "encWheelDiam")   == 0) {
        gCalib.encWheelDiamMm = val;
        gCalib.mmPerCount     = PI_F * val / (ENC_PPR * 4.0f);
    } else if (strcmp(key, "encWheelbase")  == 0) { gCalib.encWheelbase     = val; }
    else if   (strcmp(key, "wheelbase")     == 0) { gCalib.wheelbase        = val; }
    else if   (strcmp(key, "driveWheelDiam") == 0) {
        gCalib.driveWheelDiamMm = val;
        gCalib.stepsPerMm       = (float)STEPPER_STEPS_REV / (PI_F * val);
    } else if (strcmp(key, "lidarOffset")   == 0) { gCalib.lidarOffsetDeg   = val; }
    else if   (strcmp(key, "kp")            == 0) { gCalib.kp               = val; }
    else if   (strcmp(key, "ki")            == 0) { gCalib.ki               = val; }
    else if   (strcmp(key, "kd")            == 0) { gCalib.kd               = val; }
    else if   (strcmp(key, "iMax")          == 0) { gCalib.iMax             = val; }
    else if   (strcmp(key, "stopMm")        == 0) { gCalib.stopMm           = val; }
    else if   (strcmp(key, "minSpd")        == 0) { gCalib.minSpd           = val; }
    else if   (strcmp(key, "defaultSpeed")  == 0) { gCalib.defaultSpeed     = val; }
    else if   (strcmp(key, "defaultAccel")  == 0) { gCalib.defaultAccel     = val; }
    else return;

    _buildCalibJson();
    _calibNew = true;
}

#endif // WIFI_LOG_ENABLED
