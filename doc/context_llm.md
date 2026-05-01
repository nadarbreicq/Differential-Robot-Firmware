# Contexte LLM — Differential Robot Firmware

Ce document est destiné à un LLM pour comprendre rapidement le projet et aider à modifier le code.

---

## Résumé projet

Firmware ESP32-S3 pour robot différentiel de compétition (Coupe de France de Robotique).
Langage : C++17, framework Arduino + FreeRTOS, build system PlatformIO.
Moteurs : 2× pas-à-pas pilotés par FastAccelStepper. LIDAR : LD06 (série 230400 baud).

---

## Structure des fichiers

```
src/
├── main.cpp                  # Setup, loop, tâches FreeRTOS
├── config.h                  # TOUTES les constantes matériel/algo
├── display/
│   ├── oled.h                # DisplayData struct, RobotState enum, robotStateStr()
│   └── oled.cpp              # Rendu SSD1306 via U8g2, mesure CPU
├── io/
│   ├── buttons.h/cpp         # tirette(), teamSwitch(), initPressed()
│   └── leds.h/cpp            # ledsInit(), ledsSetTeam(), ledsUpdateLidar()
├── lidar/
│   ├── ld06.h/cpp            # Driver LD06 : parsing UART, buffer scan thread-safe
├── motion/
│   ├── step_control.h/cpp    # Contrôle moteurs + dead reckoning (pose X/Y/theta)
│   ├── stepper_ctrl.h/cpp    # Variante avec encodeurs (non utilisée actuellement)
│   └── encoder.h/cpp         # Interface encodeurs PCNT ESP32
└── strategy/
    ├── robot.h/cpp           # API haut niveau Robot (go/turn/gotoXY + obstacle)
    ├── strategy.h/cpp        # Fonctions stratégie utilisateur (à modifier)
```

---

## config.h — constantes clés

```c
// LIDAR
LIDAR_RX_PIN=9, LIDAR_PWM_PIN=8, LIDAR_PWM_DUTY=192 (~3600 RPM)
LIDAR_OFFSET_DEG=0.0f   // offset fin ; la correction 270°-angle est dans le code

// Moteurs (pins après échange gauche/droite suite à debug)
STEPPER_R_STEP_PIN=12, STEPPER_R_DIR_PIN=13  // moteur DROIT
STEPPER_L_STEP_PIN=11, STEPPER_L_DIR_PIN=10  // moteur GAUCHE
STEPPER_EN_PIN=21        // LOW = activé
STEPPER_L_INVERT=false, STEPPER_R_INVERT=true
// Note : GPIO13 nécessite drive strength MAX (GPIO_DRIVE_CAP_3) — bug matériel connu,
//        corrigé dans step_control.cpp::begin()

// Géométrie
DRIVE_WHEEL_DIAM_MM=57.7f  // diamètre roues de traction
WHEELBASE_MM=148.5f         // voie (distance inter-roues)
STEPS_PER_MM = 1600 / (π × 57.7) ≈ 8.82 pas/mm
ROBOT_BACK_TO_CENTER_MM=80.9f   // arrière → axe roues (recalage bordure)
TABLE_WIDTH_MM=3000  // axe X (horizontal)
TABLE_HEIGHT_MM=2000 // axe Y (vertical, vers le bas)

// Cinématique
DEFAULT_SPEED_MMS=200, DEFAULT_ACCEL_MMS2=150
TURN_SPEED_MMS=160, TURN_ACCEL_MMS2=120

// Détection obstacle
OBS_DETECT_DIST_MM=400   // zone de détection (mm)
OBS_WIDTH_MM=200          // largeur zone
OBS_MIN_DIST_MM=30        // min projection avant
LIDAR_BODY_DIST_MM=80     // filtre points intérieur robot (distance brute)
OBS_BACKUP_MM=100         // recul après détection
OBS_STOP_ACCEL_MMS2=2000  // décélération d'urgence
OBS_POLL_MS=20            // période polling obstacle
OBS_WAIT_MS=3000          // timeout dégagement adversaire
LIDAR_LED_DIST_MM=OBS_DETECT_DIST_MM  // seuil LEDs = seuil robot

// Boutons
BTN_TIRETTE_PIN=14 (INPUT, pull-up externe — HIGH=retirée)
BTN_TEAM_PIN=17   (INPUT_PULLUP — HIGH=YELLOW, LOW=BLUE)
BTN_INIT_PIN=3    (INPUT_PULLUP — LOW=pressé, front descendant)

// NeoPixel
NEO_PIN=46, NEO_COUNT=7
```

---

## Orientation LIDAR — IMPORTANT

Le LD06 est monté **tourné 90° dans le sens horaire** sur le robot et tourne dans le **sens horaire** (inversé vs convention standard).

Transformation angle LIDAR → repère robot :
```cpp
robot_angle_deg = 270.0f - lidar_angle_deg + LIDAR_OFFSET_DEG
```
Appliquée dans `robot.cpp::_obstacleSimple()`, `_obstacleWallFiltered()` et `_lidarToWorld()`.
Également dans `leds.cpp::ledsUpdateLidar()`.

Vérification :
- LIDAR 270° → robot 0° = avant ✓
- LIDAR 0°   → robot 270° = droite ✓
- LIDAR 90°  → robot 180° = arrière ✓
- LIDAR 180° → robot 90° = gauche ✓

---

## Tâches FreeRTOS

```
Core 0 : taskLidar    (prio 2) — lecture UART LD06, update buffer scan
Core 1 : taskStrategy (prio 2) — pré-match + stratégie match
Core 1 : taskDisplay  (prio 1) — rendu OLED 2 Hz
Core 1 : loop()       (prio ?) — LEDs 100ms + Teleplot Serial 500ms
```

Données partagées via `DisplayData gDisplay` (display/oled.h) — champs `volatile`, écriture atomique sur ESP32.

---

## Machine d'état pré-match (main.cpp::taskStrategy)

```
WAIT_INIT
  → teamSwitch() lu en continu → ledsSetTeam()
  → si équipe change → reset WAIT_INIT
  → initPressed() → runInitYellow/Blue() → WAIT_TIRETTE_IN

WAIT_TIRETTE_IN
  → !tirette() (LOW = insérée) → WAIT_TIRETTE_OUT

WAIT_TIRETTE_OUT
  → tirette() (HIGH = retirée) → MATCH

MATCH : runStrategyYellow() ou runStrategyBlue()
```

---

## API Robot (strategy/robot.h)

```cpp
// Déplacements bloquants (avec détection obstacle si enableObstacle())
void go(float mm);                          // + = avant, - = arrière
void turn(float deg);                       // + = gauche (CCW), - = droite (CW)
void gotoXY(float x, float y);
void gotoXY(float x, float y, float arrival_deg);

// Pose
void setPosition(float x_mm, float y_mm, float theta_deg);
float getX(), getY(), getTheta(), getThetaDeg();

// Vitesse
void setSpeed(float mmS);

// Obstacle
void enableObstacle();
void disableObstacle();
enum class DetectMode { SIMPLE, WALL_FILTERED };
void setDetectMode(DetectMode m);
// SIMPLE (défaut) : rectangle fwd/lat, pas de filtre murs
// WALL_FILTERED   : filtre bordures table (nécessite pose réelle)

// Moteurs
void disableMotors();   // EN=HIGH, robot poussable
void enableMotors();    // EN=LOW
```

### Comportement obstacle dans go()

```
1. Poll _obstacleInDir(dir) toutes les OBS_POLL_MS ms
2. Détection → softStop(OBS_STOP_ACCEL_MMS2)    // freinage agressif
3. Recul → _motion.go(-sign × OBS_BACKUP_MM)    // vitesse/accel normales
4. Attente 500ms min + _waitObstacleClear()       // max OBS_WAIT_MS
5. Reprise avec remaining + OBS_BACKUP_MM         // boucle while
```

gDisplay.obs_dist_mm et gDisplay.obs_angle_deg sont mis à jour à chaque détection.
obs_angle_deg : repère robot, 0°=avant, +90°=gauche, -90°=droite.

---

## StepControl (motion/step_control.h)

```cpp
bool begin();
void go(float mm);           // bloquant
void turn(float deg);        // bloquant
void startGo(float mm);      // non-bloquant
void startTurn(float deg);   // non-bloquant
bool isMoving();
void stop();                  // forceStop (hard)
void softStop(float accelOverride=0);  // stopMove() avec décel optionnelle
void syncPose();              // mise à jour X/Y/theta depuis pas
void setPosition(float x, float y, float theta_deg);
void setSpeed(float mmS);
void setAcceleration(float mmS2);
float getSpeed(), getAcceleration();
void disableMotors(), enableMotors();
```

Dead reckoning : pose calculée depuis les pas commandés (pas d'encodeurs).
`syncPose()` appelé après chaque mouvement bloquant.

Turn : `arc = WHEELBASE_MM × π × |deg| / 360`. Gauche : L recule, R avance.

---

## LD06 (lidar/ld06.h)

```cpp
void begin();
void update();   // à appeler en boucle (tâche Core 0)
uint16_t getScan(LidarPoint *dst, uint16_t maxPts);  // thread-safe
float getRPM();

struct LidarPoint { float angle_deg; uint16_t distance_mm; uint8_t confidence; };
```

- Protocole : paquets 47 octets, CRC-8/MAXIM
- 12 points/paquet avec interpolation angulaire
- Buffer circulaire 500 points, publié à chaque révolution complète (~16ms à 3600 RPM)
- TX serial passé comme -1 (GPIO libéré, vitesse contrôlée par PWM LEDC canal 0)

---

## Affichage OLED (display/oled.h)

```cpp
struct DisplayData {
    float pose_x_mm, pose_y_mm, pose_theta_deg;
    RobotState robot_state;
    Team team;
    float lidar_rpm;
    uint16_t lidar_pts;
    bool lidar_ok;
    uint8_t cpu0_pct, cpu1_pct;
    float obs_dist_mm, obs_angle_deg;  // obstacle le plus proche
};
extern DisplayData gDisplay;  // partagé entre toutes les tâches
```

Layouts :
- **Pré-match** (WAIT_INIT/INIT/WAIT_TIRETTE_IN/WAIT_TIRETTE_OUT) : état + équipe grande police + LIDAR status
- **OBSTACLE** : dist + angle + secteur (AVANT / AVANT GAUCHE / AVANT DROITE / ARRIERE)
- **Autres** : état + X/Y/Cap + CPU

---

## LEDs NeoPixel (io/leds.h)

```
      AVANT
  [4]  [5]  [6]
       [0]          LED 0 = équipe (ledsSetTeam)
  [3]  [2]  [1]
      ARRIÈRE
```

Secteurs (kSectors dans leds.cpp) :
```
LED 5 : 330°→30°  avant-centre   (passage par 0°)
LED 4 : 30°→90°   avant-gauche
LED 3 : 90°→150°  arrière-gauche
LED 2 : 150°→210° arrière-centre
LED 1 : 210°→270° arrière-droite
LED 6 : 270°→330° avant-droite
```
Angles en repère robot (après transformation 270°-lidar_angle).
Rafraîchissement : 100ms (loop() dans main.cpp).
Rouge = obstacle dans le secteur (<OBS_DETECT_DIST_MM), vert dim = libre.

---

## Points d'attention / bugs connus

1. **Drive strength GPIO13** : niveau HIGH insuffisant par défaut. Corrigé dans `step_control.cpp::begin()` avec `gpio_set_drive_capability(..., GPIO_DRIVE_CAP_3)` sur tous les pins moteurs.

2. **Filtre murs WALL_FILTERED** : nécessite une pose correcte. Avec `setPosition(0,0,0)` les obstacles en Y=0 sont filtrés comme murs. Utiliser `DetectMode::SIMPLE` par défaut ou donner une pose réaliste.

3. **Pins moteurs échangés** : suite à debug hardware, le moteur physiquement à droite utilise les pins 12/13 (définis comme STEPPER_R) et le gauche 11/10 (STEPPER_L). Les flags INVERT compensent.

4. **Odométrie** : dead reckoning pur (pas d'encodeurs dans la boucle de contrôle). La dérive s'accumule sur de longs trajets. Les encodeurs sont câblés mais pas utilisés (stepper_ctrl.cpp/encoder.cpp disponibles).

5. **Thread safety LEDs** : `ledsSetTeam()` (tâche strategy, core 1) et `ledsUpdateLidar()` (loop, core 1) peuvent s'interleaver. Accepté pour ce cas d'usage.

---

## Fichiers à modifier selon l'objectif

| Objectif | Fichier(s) |
|---|---|
| Calibrer le robot | `config.h` uniquement |
| Écrire la stratégie | `strategy/strategy.cpp` |
| Modifier la détection obstacle | `strategy/robot.cpp` |
| Changer l'affichage OLED | `display/oled.cpp` |
| Modifier le comportement des LEDs | `io/leds.cpp` |
| Ajouter une commande Robot | `strategy/robot.h` + `robot.cpp` |
| Modifier la cinématique | `motion/step_control.cpp` |
