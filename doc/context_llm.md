# Contexte LLM — Differential Robot Firmware

Ce document est destiné à un LLM pour comprendre rapidement le projet et aider à modifier le code.

---

## Résumé projet

Firmware ESP32-S3 pour robot différentiel de compétition (Coupe de France de Robotique).
Langage : C++17, framework Arduino + FreeRTOS, build system PlatformIO.
Moteurs : 2× pas-à-pas pilotés par FastAccelStepper.
Encodeurs : roues codeuses passives AMT102V sur PCNT ESP32-S3 — utilisés pour l'odométrie de navigation (double PID position) ET l'affichage.
LIDAR : LD06 (série 230400 baud). Actionneurs I2C : PCA9685 (servos) + PCF8574 (GPIO).

---

## Structure des fichiers

```text
src/
├── main.cpp                  # Setup, loop, tâches FreeRTOS, instances globales
├── config.h                  # Constantes matériel/algo (compile-time)
├── live_config.h/cpp         # gCalib : paramètres modifiables en RAM via Web UI
├── credentials.h             # WiFi SSID/pass (git-ignoré)
├── state_cmd.h               # Enum StateCmd (commandes UI → strategy)
├── log.h                     # Macros LOG_E/W/I/D (Serial.printf filtré par LOG_LEVEL)
├── utils.h                   # wait(ms) — délai FreeRTOS portable
├── display/
│   ├── oled.h                # DisplayData struct, RobotState enum
│   └── oled.cpp              # Rendu SSD1306 via U8g2, mesure CPU
├── io/
│   ├── buttons.h/cpp         # tirette(), teamSwitch(), initPressed()
│   └── leds.h/cpp            # ledsInit(), ledsSetTeam(), ledsUpdateLidar()
├── lidar/
│   └── ld06.h/cpp            # Driver LD06 : parsing UART, buffer scan thread-safe
├── motion/
│   ├── step_control.h/cpp    # Contrôle moteurs + dead reckoning (pose X/Y/theta)
│   ├── encoder.h/cpp         # QuadEncoder : PCNT hardware ESP32-S3
│   └── motion_ctrl.h/cpp     # MotionController : tâche PID continue (taskMotionControl)
├── wifi/
│   └── log_server.h/cpp      # WebSocket + HTTP server (UI, logs, télémétrie)
├── actuators/
│   ├── pca9685.h/cpp         # Driver PCA9685 I2C (16 canaux PWM servo)
│   ├── pcf8574.h/cpp         # Driver PCF8574 I2C (8 I/O numériques)
│   ├── servo.h/cpp           # Classe Servo (angle, %, vitesse)
│   ├── actuators.h           # Déclarations instances + séquences — À ÉDITER
│   └── actuators.cpp         # Définitions + séquences d'actionneurs — À ÉDITER
└── strategy/
    ├── robot.h/cpp           # API haut niveau Robot (wrappers de MotionController)
    ├── poi.h                 # Points d'intérêt (Vec2 + namespace POI)
    └── strategy.h/cpp        # Fonctions stratégie utilisateur — À ÉDITER

data/
├── index.html                # UI web (layout 2 colonnes + onglets)
├── field.jpg                 # Image fond de terrain
└── poi.js                    # AUTO-GÉNÉRÉ depuis poi.h (cf. scripts/sync_poi.py)

scripts/
└── sync_poi.py               # Génère data/poi.js depuis src/strategy/poi.h
                              # (lancé par PIO en extra_scripts pré-build)
```

---

## config.h — constantes clés

```c
// LIDAR
LIDAR_RX_PIN=9, LIDAR_PWM_PIN=8, LIDAR_PWM_DUTY=192 (~3600 RPM)
LIDAR_OFFSET_DEG=0.0f   // offset fin ; correction 270°-angle dans le code
LIDAR_BODY_DIST_MM=80   // filtre points < 80mm (intérieur robot)

// Zones aveugles LIDAR (poteaux structurels)
LIDAR_BLIND_L_START=75, LIDAR_BLIND_L_END=105   // poteau gauche (°, repère robot)
LIDAR_BLIND_R_START=255, LIDAR_BLIND_R_END=285   // poteau droite

// Moteurs
STEPPER_R_STEP_PIN=12, STEPPER_R_DIR_PIN=13
STEPPER_L_STEP_PIN=11, STEPPER_L_DIR_PIN=10
STEPPER_EN_PIN=21  // LOW = activé
STEPPER_L_INVERT=false, STEPPER_R_INVERT=true
// IMPORTANT : tous les pins moteur → GPIO_DRIVE_CAP_3 dans step_control.cpp::begin()

// Géométrie roues motrices
DRIVE_WHEEL_DIAM_MM=57.53f
WHEELBASE_MM=145.60f
STEPS_PER_MM = 1600/(π×57.53) ≈ 8.84

// Encodeurs (roues codeuses passives, séparées des motrices)
ENC_WHEEL_DIAM_MM=50.58f    // calibré par mesure
ENC_WHEELBASE_MM=189.0f     // voie roues codeuses ≠ WHEELBASE_MM
ENC_PPR=2048, ENC_COUNTS_PER_REV=8192
MM_PER_COUNT = π×50.58/8192 ≈ 0.01939mm
ENC_LEFT_INVERT=false, ENC_RIGHT_INVERT=true
Encodeurs : PCNT_UNIT_2 (right, pins 1/2), PCNT_UNIT_3 (left, pins 6/7)

// Dimensions robot
ROBOT_BACK_TO_CENTER_MM=80.9f
ROBOT_FRONT_TO_CENTER_MM = ROBOT_LENGTH_MM - ROBOT_BACK_TO_CENTER_MM
ROBOT_LENGTH_MM=161.8f, ROBOT_WIDTH_MM=232.0f

// Prise/dépose de stock
STOCK_TOOL_OFFSET_MM=210      // centre robot → centre stock en prise
STOCK_STAGING_MM=150          // recul avant approche finale
STOCK_DEPOSE_OFFSET_MM=180    // centre robot → centre stock en dépose
STOCK_RECAL_MM=285            // offset centre robot depuis la bordure (plaquage stock)

// Cinématique (valeurs aggressives — calibrées sur le robot)
DEFAULT_SPEED_MMS=2500, DEFAULT_ACCEL_MMS2=2500
TURN_SPEED_MMS=2500, TURN_ACCEL_MMS2=2500

// Orientations cardinales (repère table)
ANGLE_NORTH=90, ANGLE_EAST=0, ANGLE_SOUTH=270, ANGLE_WEST=180

// Table
TABLE_WIDTH_MM=3000 (X), TABLE_HEIGHT_MM=2000 (Y, vers le bas)
TABLE_MARGIN_MM=100

// Détection obstacle
OBS_DETECT_DIST_MM=400, OBS_WIDTH_MM=200, OBS_MIN_DIST_MM=60
OBS_BACKUP_MM=100, OBS_STOP_ACCEL_MMS2=2000
OBS_POLL_MS=20, OBS_WAIT_MS=3000

// Double PID position encodeurs
ENC_P1_KP=2.0     // proportionnel : mm/s par mm d'erreur
ENC_P1_KI=0.01    // intégral anti-trainage
ENC_P1_KD=0.2     // dérivé amortissement
ENC_P1_I_MAX=50   // anti-windup (mm/s)
ENC_P1_STOP_MM=5  // seuil d'arrêt translation (mm)
ENC_P1_STOP_DEG=2 // seuil d'arrêt rotation (°)
ENC_P1_MIN_SPD=8  // vitesse min moteur (mm/s)

// Logs
LOG_LEVEL=3   // 0=off 1=error 2=warn 3=info 4=debug (voir log.h)

// Chrono de match
MATCH_DURATION_MS=100000, MATCH_ENDGAME_MS=80000

// Actionneurs I2C
PCA9685_I2C_ADDR=0x40, PCF8574_I2C_ADDR=0x20
```

---

## Système de logs (log.h)

```cpp
// Inclure log.h dans tout fichier qui émet des logs
#include "log.h"   // ou "../log.h" selon le chemin

LOG_E("TAG", "erreur critique %d", code);   // niveau 1
LOG_W("TAG", "warning %.0f mm", dist);      // niveau 2
LOG_I("TAG", "position x=%.0f", x);         // niveau 3
LOG_D("TAG", "go %.0fmm -> %ld pas", mm, steps); // niveau 4
```

`LOG_LEVEL=0` en compétition : tous les blocs sont éliminés à la compilation (condition constante), zéro overhead.
Les données Teleplot (`>name:value\n`) utilisent `Serial.printf` directement — ne pas wrapper dans LOG.

---

## Orientation LIDAR

LD06 monté **90° CW** sur le robot, scan **sens horaire**.
Transformation angle LIDAR → repère robot :

```cpp
robot_angle_deg = 270.0f - lidar_angle_deg + LIDAR_OFFSET_DEG
```

Appliquée dans `robot.cpp` et `leds.cpp`. Zones aveugles LIDAR_BLIND_* appliquées après cette transformation.

---

## Repère table

Origine coin haut-gauche. X+ = droite (3000mm). Y+ = bas (2000mm).
Angle 0° = droite (Est). Positif = CCW. 90° = haut (-Y, Nord). 270° = bas (+Y, Sud).
Odométrie : `_x += dDist*cos(θ)`, `_y -= dDist*sin(θ)` (sin inversé pour Y-down).

---

## Tâches FreeRTOS

```text
Core 0 : taskLidar          (prio 2) — lecture UART LD06, update buffer scan
Core 0 : taskEncoders       (prio 3, 200 Hz) — update() + odométrie différentielle
                                                gère le rollover PCNT ESP32-S3
Core 1 : taskStrategy       (prio 2) — pré-match + match + chrono + actionneurs
Core 1 : taskMotionControl  (prio 3, 50 Hz) — PID continu, exécute les Target
                                              postés par robot.setTarget(...)
Core 1 : taskDisplay        (prio 1) — rendu OLED 2 Hz
Core 1 : LogServer task     (prio 1) — broadcast WebSocket (pose, lidar, motion...)
Core 1 : loop()                       — ledsUpdateLidar() 100ms
                                         WiFi telemetry 200ms (pose, actuateurs, motion, cc)
                                         Field click poll → robot.setTarget()
                                         Log match 500ms (LOG_LEVEL >= 3)
```

- `taskEncoders` calcule en continu `gDisplay.enc_pose_x/y/theta` utilisés par MotionController. Le mécanisme `enc_reset_pending` permet à `setPosition()` de synchroniser la pose encodeur.
- `taskMotionControl` est la **nouvelle architecture** : tâche permanente qui consomme les cibles via une struct partagée (portMUX). Voir section dédiée.

---

## Machine d'état pré-match

```text
taskStrategy démarre :
  robot.disableMotors()
  encRight.init(pins, PCNT_UNIT_2)   // APRÈS FastAccelStepper (conflit GPIO matrix)
  encLeft.init(pins, PCNT_UNIT_3)
  robot.setEncoders(&encLeft, &encRight)

WAIT_INIT
  → teamSwitch() continu → ledsSetTeam()
  → initPressed() → enableMotors() → runInitYellow/Blue() → WAIT_TIRETTE_IN

WAIT_TIRETTE_IN → tirette LOW → WAIT_TIRETTE_OUT
WAIT_TIRETTE_OUT → tirette HIGH → robot.startMatch() → runStrategyYellow/Blue()

Après stratégie :
  → attendre isEndgame() si stratégie terminée avant 80s
  → robot.startNearEnd() → runNearEndYellow/Blue()   // toujours exécuté
  → attendre isMatchOver() → disableMotors() + actuatorsDisable()
```

---

## API Robot (strategy/robot.h)

```cpp
// ── Déplacements step-based (dead reckoning, open-loop) ───────────────────
void go(float mm);                    // relatif, positif = avant
bool goStall(float mm, uint32_t timeoutMs = 3000); // go + détection blocage encodeurs
void turn(float deg);                 // relatif, positif = gauche (CCW)
void gotoXY(float x, float y);
void gotoXY(float x, float y, float arrival_deg);
void gotoXY(Vec2 poi);                // surcharge POI

// ── Navigation asservie encodeurs (via MotionController) ──────────────────
// Ces wrappers historiques sont bloquants : ils appellent setTarget()
// puis waitArrived(). Backward-compatible avec l'ancien _runWheelPID.
void goPID(float mm);                 // translation asservie
void turnPID(float deg);              // rotation asservie
void gotoXYenc(float x, float y);
void gotoXYenc(float x, float y, float arrival_deg, bool backward = false);
void gotoXYenc(Vec2 poi);             // surcharge POI
void setEncoders(QuadEncoder *l, QuadEncoder *r);

// ── API motion non-bloquante (NOUVEAU, via taskMotionControl) ─────────────
void setTarget(float x_mm, float y_mm);                            // sans theta
void setTarget(float x_mm, float y_mm, float arrival_deg,
               bool backward = false);                              // avec theta
void holdPosition();                  // → mode HOLD (cible = pose courante)
void releaseMotion();                 // → mode IDLE (moteurs libres)
bool waitArrived(float blendMm = 0.0f);  // bloquant, surveille match/endgame
MotionController::State getMotionState() const;

// ── Pose ──────────────────────────────────────────────────────────────────
void setPosition(float x, float y, float theta_deg);
// Synchronise step-based ET encodeurs. Bloquant (attend taskEncoders).

// ── Cinématique ───────────────────────────────────────────────────────────
void  setSpeed(float mmS);
void  setAcceleration(float mmS2);
void  setSpeedPct(float speedPct, float accelPct = -1.0f); // % de DEFAULT_*
void  resetSpeed();                   // revient à DEFAULT_SPEED/ACCEL_MMS

// ── Obstacle ──────────────────────────────────────────────────────────────
void enableObstacle() / disableObstacle();
void setDetectMode(DetectMode m);     // SIMPLE | WALL_FILTERED — propagé au MotionController

// ── Moteurs ───────────────────────────────────────────────────────────────
void disableMotors() / enableMotors();
// disableMotors() libère aussi MotionController (→ IDLE)

// ── Chrono ────────────────────────────────────────────────────────────────
void startMatch();
void startNearEnd();     // désactive les interruptions isEndgame() dans les mouvements
bool isEndgame()   const;   // >= MATCH_ENDGAME_MS
bool isMatchOver() const;   // >= MATCH_DURATION_MS
```

### gotoXYenc — comportement

```text
1. setTarget(x, y, arrival_deg, backward)  → MotionController prend la cible
2. waitArrived(0)                          → bloque jusqu'à convergence
3. MotionController exécute en interne : ALIGN → TRANSLATE → FINAL_TURN
```

La détection d'obstacle est gérée par MotionController pendant TRANSLATE : stop → recul de OBS_BACKUP_MM → attente dégagement → reprise vers la même cible.

---

## MotionController (motion/motion_ctrl.h)

Tâche FreeRTOS continue (50 Hz, Core 1, prio 3) qui pilote les moteurs vers une cible (X, Y, θ).

### États

```text
IDLE     → moteurs libres, tâche inactive
MOVING   → PID actif vers cible, sous-phases ALIGN → TRANSLATE → FINAL_TURN
HOLD     → PID maintien position (cible = pose au moment du holdPosition)
OBSTACLE → arrêté, attente dégagement avant reprise de la phase courante
```

### Phases internes (MOVING)

```text
ALIGN       → rotation sur place vers la direction de la cible
              (skip si déjà aligné < 3° ou dist < 1mm)
TRANSLATE   → translation vers (X, Y) avec PID wheel-level
              (détection obstacle active si Target.obstacleEn = true)
FINAL_TURN  → rotation finale vers theta (skip si Target.theta_deg = NAN)
```

### API

```cpp
struct Target {
    float x_mm, y_mm;
    float theta_deg = NAN;       // NaN = pas de contrainte orientation
    float speed = 0;             // 0 → gCalib.defaultSpeed
    float accel = 0;             // 0 → gCalib.defaultAccel
    float stopMm = 0;            // 0 → gCalib.stopMm
    bool  backward = false;
    bool  obstacleEn = true;
};

// API non-bloquante
void setTarget(const Target& t);
void holdPosition();
void release();

// API bloquante (basée sur seq pour éviter race conditions)
bool waitArrived(float blendMm = 0.0f);

// Getters debug
State getState() / Phase getPhase() / DistMm() / SpeedCap() / CmdL/R() / Target*()
uint32_t getUserSeq() / getDoneSeq()  // pour la synchro waitArrived
```

### Thread safety

- Cible partagée (`_userTgt`) protégée par `portMUX_TYPE`
- Compteurs `_userSeq` (incrémente à chaque setTarget) et `_doneSeq` (signalé par _completeTarget)
- `waitArrived` capture `expected = userSeq` au début, attend `doneSeq == expected` OU `userSeq != expected` (préempté)
- ⚠️ **Ne PAS** se fier à `state == IDLE` dans waitArrived — état initial avant que la tâche pique le nouveau seq. Le bug a été corrigé.

### setSpeedPct et vitesse

`setSpeedPct(80, 80)` fixe speed = DEFAULT×0.8, accel = DEFAULT×0.8.
`gotoXYenc` sauvegarde/restaure la vitesse en interne.
`takeStock` sauvegarde/restaure la vitesse autour du setSpeedPct(5) d'approche.
`turnPID` utilise `min(getSpeed(), TURN_SPEED_MMS)` — respecte setSpeedPct.

### _nearEndMode

Après `robot.startNearEnd()`, les checks `isEndgame()` dans `go()`, `goPID()`, `turnPID()`, `_runWheelPID()` sont désactivés. Seul `isMatchOver()` reste actif.

---

## Stratégie — fonctions disponibles (strategy.h)

```cpp
void runInitYellow(Robot &robot);    // calage X+Y contre bordures, → startYellow
void runInitBlue(Robot &robot);      // symétrique
void runStrategyYellow(Robot &robot);
void runStrategyBlue(Robot &robot);  // symétrique de yellow (axe X=1500)
void runNearEndYellow(Robot &robot); // repli fin de match (toujours exécuté à 80s)
void runNearEndBlue(Robot &robot);

// Prise/dépose de stock
void takeStock (Robot &robot, float x, float y, float angleDeg);
void takeStock (Robot &robot, Vec2 poi, float angleDeg);   // surcharge POI
void deposeStock(Robot &robot, float x, float y, float angleDeg);
void deposeStock(Robot &robot, Vec2 poi, float angleDeg);

void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR);
```

### takeStock

```text
1. gotoXYenc(stageX, stageY, angleDeg)  // staging + orientation en une étape
2. setSpeedPct(5, 5) — approche lente
   ANGLE_WEST : goStall(500) → setPosition(STOCK_RECAL_MM, y, WEST)
   ANGLE_EAST : goStall(500) → setPosition(TABLE_WIDTH - STOCK_RECAL_MM, y, EAST)
   autres     : goPID(STOCK_STAGING_MM)
   → restore speed
3. sequencePrise()
4. goPID(-50)  // dégage
```

### runInit (calage bordure)

```text
setPosition(0, 0, ANGLE_EAST)
goStall(-150)   → plaque arrière contre bordure Ouest
setPosition(ROBOT_BACK_TO_CENTER_MM, 0, ANGLE_EAST)   // X calé
goPID(375 - ROBOT_BACK_TO_CENTER_MM)   // dégage vers startYellow.x

turnPID(-90)    → face Sud
goStall(-150)   → plaque arrière contre bordure Nord
setPosition(getX(), ROBOT_BACK_TO_CENTER_MM, ANGLE_SOUTH)  // Y calé
goPID(225)      // dégage vers startYellow.y

gotoXYenc(POI::startYellow, ANGLE_SOUTH)
```

Blue : même séquence, bordure Est (getStall = goStall(-150) depuis ANGLE_WEST), turnPID(+90).

---

## POI (strategy/poi.h)

```cpp
struct Vec2 { float x, y; };

namespace POI {
    startYellow=(375,225), startBlue=(2625,225)
    stockYellow_01=(175,800),  stockBlue_01=(2825,800)
    stockYellow_02=(175,1600), stockBlue_02=(2825,1600)
    stockYellow_04=(1150,1200),stockBlue_04=(1850,1200)
    FridgeYellow_01=(1100,280), FridgeBlue_01=(1900,280)
    // + pantry_01..10, stockYellow_03, stockBlue_03, stockNinja*
}
```

Symétrie bleu/jaune : `x_bleu = TABLE_WIDTH - x_jaune`. ANGLE_WEST↔EAST, NORTH et SOUTH inchangés.

---

## Double PID position encodeurs (`MotionController::_runPidStep`)

Le PID wheel-level est désormais intégré à `MotionController` (1 itération = 1 tick de la tâche, 20 ms).

```text
outL = KP×eL + KI∫eL + KD×deL/dt   (mm/s, saturé à ±speed)
outR = KP×eR + KI∫eR + KD×deR/dt
→ setMotorVelocities(outL, outR)    // vitesse signée, gère direction par roue
Arrêt : avg = (|eL|+|eR|)/2 ≤ stopMm
Profil de freinage : speedCap = sqrt(2 × accel × max(0, |e| − stopMm))
Speed cap ramp : speedCap = min(speedCap + accel × dt, target_speed)
                 → reset à gCalib.minSpd au début de chaque phase
                 → c'est ce qui empêche le blend (cf. roadmap 1.2)
```

Phases ALIGN et FINAL_TURN utilisent `stopMm = ENC_P1_STOP_DEG × ENC_WHEELBASE_MM × π / 360`.
`goStall` (legacy, dans `robot.cpp`) détecte le blocage : delta encodeur < 0.1mm/poll × 3 polls → stop, retourne true.

---

## StepControl (motion/step_control.h)

```cpp
bool begin();
void go(float mm) / turn(float deg);   // bloquants
void startGo(mm) / startTurn(deg) / startRunOpen(mm);
void setMotorVelocities(float leftMmS, float rightMmS);  // vitesse signée par roue
void stop() / softStop(float accelOverride);
void syncPose();
void setPosition(x, y, theta_deg);
float getSpeed() / getAcceleration();
void setSpeed() / setAcceleration();
```

---

## QuadEncoder (motion/encoder.h)

```cpp
bool init(int pinA, int pinB, pcnt_unit_t unit);
void reset();
void update();          // appelé par taskEncoders à 200Hz sur Core 0
int32_t getCount();     // total cumulé (mm = count × MM_PER_COUNT)
```

**BUG CORRIGÉ** : PCNT ESP32-S3 remet à 0 sur h_lim (pas wrap). Corrigé dans `update()`.

---

## DisplayData (display/oled.h) — champs principaux

```cpp
volatile float      pose_x_mm, pose_y_mm, pose_theta_deg;  // dead reckoning
volatile RobotState robot_state;
volatile Team       team;
volatile float      lidar_rpm;
volatile bool       lidar_ok;
volatile uint8_t    cpu0_pct, cpu1_pct;
volatile float      obs_dist_mm, obs_angle_deg;
volatile uint32_t   match_start_ms;
volatile int32_t    enc_right_cnt, enc_left_cnt;    // counts bruts
// Odométrie encodeurs (calculée dans taskEncoders) :
volatile float      enc_pose_x_mm, enc_pose_y_mm;
volatile float      enc_pose_theta_deg, enc_pose_theta_rad;
volatile float      nav_dist_mm, nav_delta_deg;     // debug navigation
// Reset odométrie (setPosition → taskEncoders) :
volatile bool       enc_reset_pending;
volatile float      enc_reset_x, enc_reset_y, enc_reset_theta_deg;
```

---

## Points d'attention critiques

1. **GPIO drive strength** : `GPIO_DRIVE_CAP_3` sur tous les pins moteur dans `step_control.cpp::begin()`. Sans ça, le moteur droit ne va que dans un sens.

2. **Encodeurs init après FastAccelStepper** : dans `taskStrategy` APRÈS le démarrage du scheduler FreeRTOS. Initialiser en `setup()` conflicte avec la matrice GPIO ESP32-S3.

3. **PCNT rollover ESP32-S3** : le compteur PCNT se remet à 0 (pas wrappe) sur h_lim/l_lim. Corrigé dans `encoder.cpp::update()`.

4. **taskEncoders sur Core 0** (prio 3) : garantit `update()` toutes les 5ms. Ne pas déplacer dans `loop()`.

5. **ENC_WHEELBASE_MM ≠ WHEELBASE_MM** : 189mm vs 145.6mm. `MotionController` utilise `gCalib.encWheelbase` pour les cibles encodeurs (phases ALIGN/FINAL_TURN). `startTurn` utilise `gCalib.wheelbase` pour les pas.

6. **setPosition() est bloquant** : attend que `taskEncoders` applique le reset (max 5ms). Ne pas appeler depuis une ISR.

7. **Obstacle désactivé pendant ALIGN et FINAL_TURN** : dans `MotionController::_initAlignPhase/_initFinalTurnPhase`, `_phaseObstacleEn = false`. La direction de détection serait fausse pendant une rotation.

8. **_nearEndMode** : appeler `robot.startNearEnd()` avant `runNearEndYellow/Blue()` pour que les mouvements fonctionnent après 80s.

9. **LOG_LEVEL=0 en compétition** : met dans `config.h`. Élimine tous les Serial.printf de log à la compilation.

10. **`waitArrived` utilise des seq, PAS l'état** : ne PAS ajouter `if (state == IDLE) return true` dans `waitArrived` — l'état initial est IDLE avant que la tâche pique le nouveau seq. Confond preempt et complétion.

11. **gCalib vs config.h** : `gCalib` contient les paramètres modifiables à chaud (PID, vitesses, géométrie). Initialisé depuis les `#define` de `config.h` au boot. La Web UI peut les modifier en RAM. Une fois validés, **les copier dans `config.h`** pour la persistance.

12. **setSpeedPct** : multiplie `gCalib.defaultSpeed/defaultAccel`, pas les constantes `DEFAULT_*_MMS` directement. La calibration live modifie donc le comportement de `setSpeedPct`.

13. **scripts/sync_poi.py** : invoqué automatiquement par PIO en `pre:` extra_script. Régénère `data/poi.js` depuis `src/strategy/poi.h`. Ajouter un POI : éditer **uniquement** `poi.h`, puis ajouter une entrée dans `POI_META` (index.html) pour le label/couleur.

---

## Live config (live_config.h/cpp)

Struct `gCalib` (globale) — paramètres modifiables en RAM via la Web UI ou les commandes WebSocket `{"type":"calib","key":"...","val":...}`. Initialisée au boot depuis les `#define` de `config.h`.

```cpp
struct LiveCalib {
    // Mécanique
    float encWheelDiam, encWheelbase, wheelbase, driveWheelDiam, lidarOffsetDeg;
    float stepsPerMm, mmPerCount;          // dérivés (recalculés)
    // PID
    float kp, ki, kd, iMax, stopMm, minSpd, defaultSpeed, defaultAccel;
};
extern LiveCalib gCalib;
```

Le code lit `gCalib.kp` au lieu de `ENC_P1_KP` etc. Une fois calibré sur le robot via UI, copier les valeurs validées dans `config.h` pour la persistance (sinon perdues au reboot).

---

## Web UI (data/index.html, log_server.cpp)

Layout 2 colonnes :

- **Gauche** : terrain canvas 900×600 (responsive, aspect 3/2) + toolbar (LIDAR terrain toggle, coords souris) + barre pose + panneau ROBOT (7 boutons d'état)
- **Droite (440 px)** : onglets MOTION / ACTIONNEURS / CALIBRATION / LOGS

### Click + drag goto (sécurisé)

- Clic simple → `goto(x, y)` sans contrainte d'orientation
- Clic-drag-relâche → `goto(x, y, θ)` avec flèche de preview rouge
- Autorisé uniquement après init et hors match en cours (`cc:1` dans pose WS)
- Message WS : `{"type":"goto","x":1234,"y":567,"t":90}` (clé `t` optionnelle)

### Boutons ROBOT (7)

```text
⏹ Stop moteurs    | ⚡ Activer moteurs | ↩ WAIT_INIT
▶ Lancer Init     | ▶ Lancer Match    | ⏱ Stop match | ↺ Relancer Match
```

Commandes via `StateCmd` enum (`src/state_cmd.h`) → `gStateCmd` queue → consommée par `taskStrategy`.

### Marqueur cible + Motion debug

- Cible MotionController affichée sur le terrain (cercle vert + flèche θ si imposé)
- Panneau MOTION : état/phase/dist/cap/V L/R/cible — mise à jour 200 ms

### POI auto-générés

`data/poi.js` régénéré par `scripts/sync_poi.py` à chaque build (PIO `pre:` extra_script). Les coordonnées viennent de `poi.h`. Les métadonnées d'affichage (label court, catégorie) restent dans `index.html::POI_META`.

---

## Fichiers à modifier selon l'objectif

| Objectif | Fichier(s) |
| --- | --- |
| Calibrer le robot (compile-time) | `config.h` |
| Calibrer à chaud (RAM, via UI) | UI web → `gCalib` (puis report dans `config.h`) |
| Écrire la stratégie | `strategy/strategy.cpp` |
| Ajouter un servo / séquence | `actuators/actuators.h` + `actuators.cpp` |
| Modifier la détection obstacle | `motion/motion_ctrl.cpp` (`_obstacleSimple/_obstacleWallFiltered`) |
| Zones aveugles LIDAR | `config.h` (LIDAR_BLIND_*) |
| Changer l'affichage OLED | `display/oled.cpp` |
| Modifier le comportement des LEDs | `io/leds.cpp` |
| Ajouter une commande Robot | `strategy/robot.h` + `robot.cpp` |
| Modifier le PID / phases motion | `motion/motion_ctrl.cpp` |
| Modifier la cinématique step-based | `motion/step_control.cpp` |
| Calibration géométrique | `strategy/strategy.cpp` + `main.cpp` |
| Verbosité des logs | `config.h` → `LOG_LEVEL` |
| Ajouter des POI | `strategy/poi.h` (UI mis à jour auto via sync_poi.py) |
| Modifier l'UI web | `data/index.html` (+ `log_server.cpp` pour nouveaux messages WS) |
| Ajouter une commande UI → robot | `state_cmd.h` + `log_server.cpp::_handleWsMsg` + `main.cpp::taskStrategy` |
