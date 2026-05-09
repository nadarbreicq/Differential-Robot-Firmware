# Contexte LLM — Differential Robot Firmware

Ce document est destiné à un LLM pour comprendre rapidement le projet et aider à modifier le code.

---

## Résumé projet

Firmware ESP32-S3 pour robot différentiel de compétition (Coupe de France de Robotique).
Langage : C++17, framework Arduino + FreeRTOS, build system PlatformIO.
Moteurs : 2× pas-à-pas pilotés par FastAccelStepper (dead reckoning pur).
Encodeurs : roues codeuses passives AMT102V sur PCNT ESP32-S3 — utilisés pour affichage/calibration uniquement, pas pour l'odométrie de navigation.
LIDAR : LD06 (série 230400 baud). Actionneurs I2C : PCA9685 (servos) + PCF8574 (GPIO).

---

## Structure des fichiers

```
src/
├── main.cpp                  # Setup, loop, tâches FreeRTOS, instances globales
├── config.h                  # TOUTES les constantes matériel/algo
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
│   └── encoder.h/cpp         # QuadEncoder : PCNT hardware ESP32-S3
├── actuators/
│   ├── pca9685.h/cpp         # Driver PCA9685 I2C (16 canaux PWM servo)
│   ├── pcf8574.h/cpp         # Driver PCF8574 I2C (8 I/O numériques)
│   ├── servo.h/cpp           # Classe Servo (angle, %, vitesse)
│   ├── actuators.h           # Déclarations instances + séquences — À ÉDITER
│   └── actuators.cpp         # Définitions + séquences d'actionneurs — À ÉDITER
└── strategy/
    ├── robot.h/cpp           # API haut niveau Robot (go/turn/gotoXY/obstacle/chrono)
    ├── strategy.h/cpp        # Fonctions stratégie utilisateur — À ÉDITER
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

// Moteurs — pins échangés suite à debug hardware
STEPPER_R_STEP_PIN=12, STEPPER_R_DIR_PIN=13
STEPPER_L_STEP_PIN=11, STEPPER_L_DIR_PIN=10
STEPPER_EN_PIN=21  // LOW = activé
STEPPER_L_INVERT=false, STEPPER_R_INVERT=true
// IMPORTANT : GPIO13 → GPIO_DRIVE_CAP_3 dans step_control.cpp::begin()

// Géométrie roues motrices
DRIVE_WHEEL_DIAM_MM=57.7f
WHEELBASE_MM=148.5f
STEPS_PER_MM = 1600/(π×57.7) ≈ 8.82

// Encodeurs (roues codeuses passives, séparées des motrices)
ENC_WHEEL_DIAM_MM=50.0f
ENC_WHEELBASE_MM=189.0f     // voie roues codeuses ≠ WHEELBASE_MM
ENC_PPR=2048, ENC_COUNTS_PER_REV=8192
MM_PER_COUNT = π×50/8192 ≈ 0.01917mm
ENC_LEFT_INVERT=false, ENC_RIGHT_INVERT=true
Encodeurs : PCNT_UNIT_2 (right, pins 1/2), PCNT_UNIT_3 (left, pins 6/7)

// Dimensions robot
ROBOT_BACK_TO_CENTER_MM=80.9f
ROBOT_LENGTH_MM=161.8f, ROBOT_WIDTH_MM=232.0f

// Prise/dépose de stock
STOCK_TOOL_OFFSET_MM=210      // centre robot → centre stock en prise
STOCK_STAGING_MM=150          // recul avant approche finale
STOCK_DEPOSE_OFFSET_MM=180    // centre robot → centre stock en dépose

// Cinématique
DEFAULT_SPEED_MMS=400, DEFAULT_ACCEL_MMS2=200
TURN_SPEED_MMS=200, TURN_ACCEL_MMS2=100

// Table
TABLE_WIDTH_MM=3000 (X), TABLE_HEIGHT_MM=2000 (Y, vers le bas)
TABLE_MARGIN_MM=100

// Détection obstacle
OBS_DETECT_DIST_MM=400, OBS_WIDTH_MM=200, OBS_MIN_DIST_MM=60
OBS_BACKUP_MM=100, OBS_STOP_ACCEL_MMS2=2000
OBS_POLL_MS=20, OBS_WAIT_MS=3000
LIDAR_LED_DIST_MM=OBS_DETECT_DIST_MM

// Chrono de match
MATCH_DURATION_MS=100000, MATCH_ENDGAME_MS=80000

// Actionneurs I2C
PCA9685_I2C_ADDR=0x40, PCF8574_I2C_ADDR=0x20
```

---

## Orientation LIDAR

LD06 monté **90° CW** sur le robot, scan **sens horaire**.
Transformation angle LIDAR → repère robot :
```cpp
robot_angle_deg = 270.0f - lidar_angle_deg + LIDAR_OFFSET_DEG
```
Appliquée dans `robot.cpp` et `leds.cpp`. Zones aveugles LIDAR_BLIND_* appliquées après cette transformation dans les deux fichiers.

---

## Repère table

Origine coin haut-gauche. X+ = droite (3000mm). Y+ = bas (2000mm).
Angle 0° = droite. Positif = CCW. 90° = haut (-Y). 270° = bas (+Y).
Odométrie : `_x += dDist*cos(θ)`, `_y -= dDist*sin(θ)` (sin inversé pour Y-down).
gotoXY : `atan2f(-dy, dx)`. _lidarToWorld : `wy = getY() - d*sinf(a)`.

---

## Tâches FreeRTOS

```
Core 0 : taskLidar    (prio 2) — lecture UART LD06, update buffer scan
Core 0 : taskEncoders (prio 3) — encLeft/Right.update() à 200Hz (5ms)
                                  gère le rollover PCNT ESP32-S3
Core 1 : taskStrategy (prio 2) — pré-match + match + chrono + actionneurs
Core 1 : taskDisplay  (prio 1) — rendu OLED 2 Hz
Core 1 : loop()               — ledsUpdateLidar() 100ms
                                 Teleplot LIDAR 200ms (WAIT_INIT seulement)
                                 Teleplot pose 500ms (match seulement)
```

Données partagées : `DisplayData gDisplay` (volatile, écriture atomique ESP32).

`taskEncoders` sur Core 0 à priorité 3 garantit que update() est appelé toutes les 5ms indépendamment de la charge de Core 1. Critique pour la gestion correcte du rollover PCNT.

---

## Machine d'état pré-match

```
taskStrategy démarre :
  robot.disableMotors()
  encRight.init(pins, PCNT_UNIT_2)   // APRÈS FastAccelStepper (conflit GPIO matrix)
  encLeft.init(pins, PCNT_UNIT_3)

WAIT_INIT
  → teamSwitch() continu → ledsSetTeam()
  → initPressed() → enableMotors() → runInitYellow/Blue() → WAIT_TIRETTE_IN
  (pendant ce temps taskEncoders tourne → OLED affiche R/L mm)

WAIT_TIRETTE_IN → tirette LOW → WAIT_TIRETTE_OUT
WAIT_TIRETTE_OUT → tirette HIGH → robot.startMatch() → runStrategyYellow/Blue()

Après stratégie :
  → si endgame → runNearEndYellow/Blue()
  → attendre isMatchOver() → disableMotors() + actuatorsDisable()
```

---

## API Robot (strategy/robot.h)

```cpp
// Déplacements bloquants (dead reckoning)
void go(float mm);
void turn(float deg);
void gotoXY(float x, float y);
void gotoXY(float x, float y, float arrival_deg);

// Pose
void setPosition(float x, float y, float theta_deg);

// Vitesse
void setSpeed(float mmS);

// Obstacle
void enableObstacle() / disableObstacle();
enum DetectMode { SIMPLE, WALL_FILTERED };
void setDetectMode(DetectMode m);

// Moteurs
void disableMotors() / enableMotors();

// Chrono
void startMatch();
bool isEndgame()   const;   // >= MATCH_ENDGAME_MS
bool isMatchOver() const;   // >= MATCH_DURATION_MS
uint32_t matchElapsed() const;
```

### Comportement go() avec obstacle et chrono

```
Boucle polling (OBS_POLL_MS = 20ms) :
  1. _motion.syncPose()    → dead reckoning depuis pas commandés
  2. isMatchOver()         → softStop + disableMotors + vTaskDelete
  3. isEndgame()           → softStop + break
  4. _obstacleInDir(dir)   → softStop + recul + attente + reprise
```

---

## Stratégie — fonctions disponibles (strategy.h)

```cpp
void runInitYellow(Robot &robot);
void runInitBlue(Robot &robot);
void runStrategyYellow(Robot &robot);
void runStrategyBlue(Robot &robot);
void runNearEndYellow(Robot &robot);   // repli fin de match
void runNearEndBlue(Robot &robot);

// Prise/dépose de stock (géométrie paramétrée)
void takeStock (Robot &robot, float x, float y, float angleDeg);
void deposeStock(Robot &robot, float x, float y, float angleDeg);

// Calibration géométrique (à brancher temporairement dans main.cpp)
void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR);
```

### takeStock / deposeStock

```
takeStock(robot, x, y, angleDeg) :
  1. gotoXY(stageX, stageY)          // staging = pick - STAGING_MM dans approche
  2. gotoXY(stageX, stageY, angleDeg) // orientation
  3. go(STOCK_STAGING_MM)             // avance vers position de prise
  4. sequencePrise()

deposeStock(robot, x, y, angleDeg) :
  1. gotoXY(depX, depY, angleDeg)    // centre robot à STOCK_DEPOSE_OFFSET_MM du stock
  2. ouvrirGripper() → wait → go(-100) → fermerGripper()
```

---

## StepControl (motion/step_control.h)

```cpp
bool begin();
void go(float mm) / turn(float deg);   // bloquants
void startGo(mm) / startTurn(deg);     // non-bloquants
void stop() / softStop(float accelOverride);
void syncPose();                        // MAJ pose depuis pas commandés, WHEELBASE_MM
void setPosition(x, y, theta_deg);
void disableMotors() / enableMotors();
float getSpeed() / getAcceleration();
void setSpeed() / setAcceleration();
```

Dead reckoning pur : pose depuis pas commandés. Pas d'encodeurs dans la boucle de contrôle.

---

## QuadEncoder (motion/encoder.h)

```cpp
bool init(int pinA, int pinB, pcnt_unit_t unit);
void reset();
void update();          // appelé par taskEncoders à 200Hz sur Core 0
int32_t getCount();     // total cumulé (mm = count × MM_PER_COUNT)
```

**BUG CORRIGÉ** : Sur ESP32-S3, le compteur PCNT se remet à 0 quand il atteint h_lim (32767) — il ne wrappe PAS en -32768 comme l'int16_t standard. L'implémentation de `update()` détecte ce cas et calcule le bon delta :
- Si `_lastHw > 16384 && hw < (_lastHw - 16384)` → reset depuis h_lim
- Si `_lastHw < -16384 && hw > (_lastHw + 16384)` → reset depuis l_lim

Sans ce fix, un mouvement > 628mm cause une massive soustraction dans `_totalCount`.

Encodeurs utilisés pour :
- Affichage WAIT_INIT (gDisplay.enc_right/left_cnt)
- Calibration géométrique (runCalibration)
- PAS pour l'odométrie de navigation (dead reckoning seulement)

---

## utils.h

```cpp
inline void wait(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
```

Équivalent FreeRTOS de `delay()`. Inclus dans `actuators.h` et `strategy.cpp`.

---

## Actionneurs (actuators/)

### Servo
```cpp
struct ServoConfig { uint8_t channel; uint16_t minUs, maxUs; float minDeg, maxDeg; };
void setAngle(float deg);
void setPercent(float pct);
void moveTo(float deg, float degPerSec);       // bloquant
void moveToPercent(float pct, float pctPerSec);
void detach();
```

### actuators.h / actuators.cpp — fichiers utilisateur
`actuatorsInit()` appelé dans setup(). `actuatorsDisable()` à la fin du match.

---

## Calibration géométrique (strategy/strategy.h)

```cpp
void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR);
```

**Angle** : `turn(720)` (2 tours) → mesure (arcD - arcL) / ENC_WHEELBASE_MM → calcule WHEELBASE_MM.
Formule : `WHEELBASE_new = WHEELBASE × (720 / angle_mesuré)`.
Attention : la formule est `720 / angle` (PAS `angle / 720`) — si angle < 720° il faut augmenter WHEELBASE.

Pour activer : dans `main.cpp::taskStrategy`, remplacer `runInitYellow/Blue(robot)` par `runCalibration(robot, encLeft, encRight)`.

---

## DisplayData (display/oled.h)

```cpp
struct DisplayData {
    float      pose_x_mm, pose_y_mm, pose_theta_deg;
    RobotState robot_state;
    Team       team;
    float      lidar_rpm;
    uint16_t   lidar_pts;
    bool       lidar_ok;
    uint8_t    cpu0_pct, cpu1_pct;
    float      obs_dist_mm, obs_angle_deg;
    uint32_t   match_start_ms;
    int32_t    enc_right_cnt, enc_left_cnt;  // affichage WAIT_INIT uniquement
};
```

RobotState : WAIT_INIT → INIT → WAIT_TIRETTE_IN → WAIT_TIRETTE_OUT → MOVING/TURNING/GOTO/OBSTACLE/ENDGAME → DONE

---

## Points d'attention critiques

1. **GPIO13 drive strength** : GPIO_DRIVE_CAP_3 obligatoire (step_control.cpp::begin()). Sans ça, le moteur droit ne va que dans un sens.

2. **Encodeurs init après FastAccelStepper** : dans taskStrategy APRÈS le démarrage du scheduler FreeRTOS. Initialiser en setup() conflicte avec la matrice GPIO ESP32-S3.

3. **PCNT rollover ESP32-S3** : le compteur PCNT se remet à 0 (pas wrappe) sur h_lim/l_lim. Corrigé dans encoder.cpp::update(). Sans le fix, les comptages > 628mm sont faux.

4. **taskEncoders sur Core 0** (prio 3) : garantit update() toutes les 5ms. Ne pas déplacer dans loop() — loop() peut être préempté par la tâche stratégie et ne pas tourner assez vite.

5. **ENC_WHEELBASE_MM ≠ WHEELBASE_MM** : 189mm vs 148.5mm. Utiliser le bon selon le contexte (calibration angle vs commande rotation).

6. **Calibration WHEELBASE** : formule `WHEELBASE × (720/angle)`, PAS `WHEELBASE × (angle/720)`. Erreur inverse = empire la calibration.

7. **Filtre murs WALL_FILTERED** : nécessite pose correcte. Utiliser DetectMode::SIMPLE (défaut).

8. **Wire.begin()** dans setup() en premier. oled.cpp ne l'appelle plus.

9. **Teleplot LIDAR** : uniquement en WAIT_INIT (200ms). Désactivé pendant match.

---

## Fichiers à modifier selon l'objectif

| Objectif | Fichier(s) |
|---|---|
| Calibrer le robot | `config.h` |
| Écrire la stratégie | `strategy/strategy.cpp` |
| Ajouter un servo / séquence | `actuators/actuators.h` + `actuators.cpp` |
| Modifier la détection obstacle | `strategy/robot.cpp` |
| Zones aveugles LIDAR | `config.h` (LIDAR_BLIND_*) |
| Changer l'affichage OLED | `display/oled.cpp` |
| Modifier le comportement des LEDs | `io/leds.cpp` |
| Ajouter une commande Robot | `strategy/robot.h` + `robot.cpp` |
| Modifier la cinématique | `motion/step_control.cpp` |
| Calibration géométrique | `strategy/strategy.cpp` + `main.cpp` |
