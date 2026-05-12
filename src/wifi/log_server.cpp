#include "log_server.h"

#if WIFI_LOG_ENABLED

#include <WiFi.h>
#include <LittleFS.h>
#include "../credentials.h"
#include "../config.h"

LogServer logServer;

// ─── Fonctions libres ─────────────────────────────────────────────────────────

void wifiLogBegin()  { logServer.begin(); }

void wifiLogPush(char level, const char* tag, const char* msg) {
    logServer.push(level, tag, msg);
}

void wifiLogUpdatePose(float x, float y, float theta_deg, const char* state,
                       float nav_dist_mm, float nav_delta_deg, char team) {
    logServer.updatePose(x, y, theta_deg, state, nav_dist_mm, nav_delta_deg, team);
}

void wifiLogLidar(const LidarPoint* buf, uint16_t n) {
    logServer.updateLidar(buf, n);
}

void wifiLogUpdateActuators(float bd, float bg, float li, float gr) {
    logServer.updateActuators(bd, bg, li, gr);
}

bool wifiPollCmd(ServoCmd& out) {
    return logServer.pollCmd(out);
}

// ─── begin ────────────────────────────────────────────────────────────────────

void LogServer::begin() {
    _logQueue = xQueueCreate(64, sizeof(LogEntry));
    _cmdQueue = xQueueCreate(8,  sizeof(ServoCmd));

    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[I][WIFI] AP: %s  IP: %s\n", WIFI_AP_SSID,
                  WiFi.softAPIP().toString().c_str());

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
    _http.onNotFound([this]() { _http.send(404, "text/plain", "Not found"); });
    _http.begin();

    _ws.onEvent([this](uint8_t, WStype_t type, uint8_t* payload, size_t len) {
        if (type == WStype_TEXT && len > 4)
            _handleWsMsg(payload, len);
    });
    _ws.begin();

    xTaskCreatePinnedToCore(_task, "logwifi", TASK_WIFI_STACK, this,
                            TASK_WIFI_PRIO, nullptr, TASK_WIFI_CORE);
}

// ─── Réception WebSocket → commandes actionneurs ──────────────────────────────

void LogServer::_handleWsMsg(uint8_t* payload, size_t len) {
    // Format attendu : {"type":"servo","id":"brasDroit","val":65.0}
    const char* p = (const char*)payload;
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
                           float nav_dist_mm, float nav_delta_deg, char team) {
    _px = x; _py = y; _ptheta = theta_deg;
    _navDist = nav_dist_mm; _navDelta = nav_delta_deg;
    _team = team;
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

// ─── updateActuators ──────────────────────────────────────────────────────────

void LogServer::updateActuators(float bd, float bg, float li, float gr) {
    _actBd = bd; _actBg = bg; _actLi = li; _actGr = gr;
    _actNew = true;
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
            char buf[140];
            snprintf(buf, sizeof(buf),
                     "{\"type\":\"pose\",\"x\":%.1f,\"y\":%.1f,\"a\":%.1f,"
                     "\"state\":\"%s\",\"nd\":%.0f,\"na\":%.1f,\"tm\":\"%c\"}",
                     (double)_px, (double)_py, (double)_ptheta,
                     _pstate, (double)_navDist, (double)_navDelta, (char)_team);
            _ws.broadcastTXT(buf);
        }

        // LIDAR
        if (_lidarNew) {
            _lidarNew = false;
            _ws.broadcastTXT(_lidarJson);
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif // WIFI_LOG_ENABLED
