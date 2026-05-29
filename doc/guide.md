# Guide d'utilisation — Differential Robot Firmware

## Vue d'ensemble

Firmware pour robot différentiel de compétition (Coupe de France de Robotique) basé sur un **ESP32-S3**. Il gère la motorisation pas-à-pas, le LIDAR LD06, la détection d'adversaire, l'affichage OLED, les LEDs NeoPixel, les actionneurs I2C et les roues codeuses (affichage + calibration).

---

## Matériel requis

| Composant | Détail |
|---|---|
| Microcontrôleur | ESP32-S3-DevKitC-1 |
| Moteurs | 2× pas-à-pas (200 pas/tr, 8 micropas) |
| Encodeurs | 2× AMT102V sur roues codeuses ∅50mm, 2048 PPR |
| LIDAR | LD06 |
| Écran | OLED SSD1306 128×64 I2C |
| LEDs | NeoPixel × 7 (pin 46) |
| Actionneurs servo | PCA9685 I2C (0x40) — 16 canaux PWM |
| I/O numérique | PCF8574 I2C (0x20) — 8 broches |
| Tirette | Pin 14, pull-up externe |
| Switch équipe | Pin 17, pull-up interne |
| Bouton init | Pin 3, pull-up interne |

---

## Démarrage rapide

### 1. Configurer le matériel — `src/config.h`

```c
// Roues motrices
#define DRIVE_WHEEL_DIAM_MM      57.7f   // diamètre réel (pied à coulisse)
#define WHEELBASE_MM            148.5f   // distance entre roues motrices

// Roues codeuses (passives, séparées des motrices — pour test et calibration)
#define ENC_WHEEL_DIAM_MM        50.0f
#define ENC_WHEELBASE_MM        189.0f   // distance entre roues codeuses

// Dimensions robot
#define ROBOT_BACK_TO_CENTER_MM  80.9f   // arrière → axe des roues

// Vitesse et accélération
#define DEFAULT_SPEED_MMS       400.0f   // mm/s
#define DEFAULT_ACCEL_MMS2      200.0f   // mm/s²

// Détection adversaire
#define OBS_DETECT_DIST_MM      400.0f   // mm

// Zones aveugles LIDAR (poteaux structurels du robot)
#define LIDAR_BLIND_L_START      75.0f   // début zone gauche (°, repère robot)
#define LIDAR_BLIND_L_END       105.0f
#define LIDAR_BLIND_R_START     255.0f   // début zone droite
#define LIDAR_BLIND_R_END       285.0f

// Chrono de match
#define MATCH_DURATION_MS      100000UL  // 100 s
#define MATCH_ENDGAME_MS        80000UL  // repli à 80 s
```

### 2. Flasher et démarrer

```bash
pio run --target upload
pio device monitor
```

---

## Séquence pré-match

```
Démarrage → moteurs DÉSENGAGÉS (robot poussable librement)

1. APPUYER INIT   → positionner le robot, appuyer sur INIT
                    → le robot effectue le calage et mémorise sa position

2. INSERER TIRETTE → insérer la tirette (pin 14 LOW)

3. PRET !          → le robot attend le retrait de la tirette

4. [Tirette retirée] → match lancé ! (chrono démarre)
```

Pendant WAIT_INIT, l'OLED affiche les **valeurs encodeurs** (R et L en mm) pour vérifier les roues codeuses.
Le **switch d'équipe** peut être changé à tout moment avant l'étape 1.

---

## Chrono de match

| Temps | Comportement |
|---|---|
| 0 → 80 s | Stratégie normale |
| 80 s | `go()`/`turn()` s'arrêtent → `runNearEndYellow/Blue()` lancé |
| 100 s | Moteurs et actionneurs désengagés |

---

## Écrire une stratégie — `src/strategy/strategy.cpp`

### Commandes robot

```cpp
// ─── Déplacements step-based (open-loop, dead reckoning) ────────────────────
robot.go(500);              // avance 500 mm (négatif = recule)
robot.turn(90);             // tourne 90° à gauche (négatif = droite)
robot.gotoXY(1000, 500);    // va aux coordonnées absolues
robot.gotoXY(1000, 500, 0); // idem avec cap final 0°
robot.goStall(-500);        // recule + stoppe sur blocage (plaquage bordure)

// ─── Déplacements asservis encodeurs (closed-loop, recommandé) ──────────────
robot.goPID(500);                       // translation asservie
robot.turnPID(90);                      // rotation asservie
robot.gotoXYenc(1000, 500);             // navigue + arrête à la cible
robot.gotoXYenc(1000, 500, 90);         // avec orientation finale
robot.gotoXYenc(POI::startYellow, 90);  // via POI nommé

// ─── API motion non-bloquante (taskMotionControl) ───────────────────────────
robot.setTarget(1000, 500, 90);   // poste la cible, retour immédiat
robot.waitArrived(0);              // bloquant : attend convergence
robot.waitArrived(150);            // sort à 150 mm de la cible (préparation blend)
robot.holdPosition();              // mode HOLD : maintient la pose actuelle
robot.releaseMotion();             // mode IDLE : moteurs libres

// ─── Pose ───────────────────────────────────────────────────────────────────
robot.setPosition(x, y, theta_deg);

// ─── Vitesse ────────────────────────────────────────────────────────────────
robot.setSpeed(400);
robot.setSpeedPct(80, 80);   // % de gCalib.defaultSpeed/defaultAccel
robot.resetSpeed();          // revient à gCalib.defaultSpeed/defaultAccel

// ─── Détection adversaire ───────────────────────────────────────────────────
robot.enableObstacle();
robot.disableObstacle();

// ─── Moteurs ────────────────────────────────────────────────────────────────
robot.disableMotors();   // libère aussi MotionController (→ IDLE)
robot.enableMotors();

// ─── Chrono ─────────────────────────────────────────────────────────────────
robot.isEndgame()    // true après MATCH_ENDGAME_MS
robot.isMatchOver()  // true après MATCH_DURATION_MS
robot.waitMatchTime(95000);   // bloque jusqu'à la 95e seconde (ou fin match)

// ─── Délai FreeRTOS (dans strategy.cpp ou actuators.cpp) ────────────────────
#include "../utils.h"
wait(500);           // attend 500ms en libérant le CPU
```

**Recommandation** : utiliser `gotoXYenc` / `goPID` / `turnPID` pour la navigation. Les variantes `go` / `turn` sont open-loop (pas de feedback encodeurs), utiles seulement pour `goStall` (plaquage bordure).

### Repère de coordonnées

```
  (0,0)──────────────────────────────→ X+  (3000 mm)
    │
    │         90°
    │          ↑
    │   180° ←─┼─→ 0°
    │          ↓
    │         270°
    ↓
   Y+  (2000 mm)
```

- **Origine** : coin haut-gauche de la table
- **X+** = droite, **Y+** = bas, **0°** = face à droite
- Angles positifs = anti-horaire : 90° = haut (-Y), 270° = bas (+Y)

### Prise et dépose de stock

```cpp
// x, y = centre du stock, angleDeg = direction d'approche
takeStock (robot, 500, 800, 90);   // approche, avance 210mm, séquencePrise()
deposeStock(robot, 200, 600, 180); // navigue, ouvre gripper, recule, ferme

// Constantes géométriques (config.h) :
// STOCK_TOOL_OFFSET_MM  = 210mm  (centre robot → centre stock en prise)
// STOCK_STAGING_MM      = 150mm  (recul avant approche finale)
// STOCK_DEPOSE_OFFSET_MM= 180mm  (centre robot → centre stock en dépose)
```

### Init — recalage bordure

```cpp
void runInitYellow(Robot &robot) {
    robot.enableMotors();
    robot.disableObstacle();
    initActuators();
    robot.go(-300);
    robot.setPosition(ROBOT_BACK_TO_CENTER_MM, 900, 0);
}
```

---

## Calibration géométrique

Disponible via `runCalibration(robot, encLeft, encRight)` dans strategy.h.
Pour l'activer temporairement : dans `main.cpp`, remplacer `runInitYellow/Blue(robot)` par `runCalibration(robot, encLeft, encRight)`.

### Calibrer DRIVE_WHEEL_DIAM_MM (distances)

Appuie sur init → robot fait `go(1000)` → mesure encodeurs → affiche la correction dans le moniteur série.

### Calibrer WHEELBASE_MM (angles)

Robot fait `turn(720)` (2 tours) → mesure encodeurs → calcule le WHEELBASE_MM corrigé.

**Formule** : `WHEELBASE_new = WHEELBASE × (720 / angle_mesuré)`
- angle mesuré < 720° → WHEELBASE trop petit → valeur augmente ✓
- angle mesuré > 720° → WHEELBASE trop grand → valeur diminue ✓

---

## Actionneurs I2C — `src/actuators/`

### Servomoteurs (PCA9685)

```cpp
// actuators.h — déclarer :
extern Servo servoBras;
void deployerBras();

// actuators.cpp — définir :
Servo servoBras(pca, {0, 500, 2500, 0, 180});  // {canal, minUs, maxUs, minDeg, maxDeg}
void deployerBras() { servoBras.moveTo(150, 60); }  // 150° à 60°/s
```

Commandes servo :
```cpp
servoBras.setAngle(90);           // direct
servoBras.setPercent(50);         // 0-100% de la course
servoBras.moveTo(90, 45);         // avec vitesse (bloquant)
servoBras.moveToPercent(100, 30);
servoBras.detach();               // relâche le PWM
```

### PCF8574 (I/O numériques)

```cpp
pcf.setPin(0, false);    // LOW = actif
pcf.setPin(0, true);     // HIGH = repos
bool e = pcf.getPin(3);
```

### Délais dans les séquences

```cpp
#include "../utils.h"  // ou via actuators.h qui l'inclut déjà
wait(1000);            // 1 seconde FreeRTOS (libère le CPU)
```

---

## Zones aveugles LIDAR

Le LIDAR peut détecter les poteaux structurels du robot. Filtrer avec :

```c
// config.h — angles en repère robot (0°=avant, 90°=gauche, 270°=droite)
#define LIDAR_BLIND_L_START  75.0f
#define LIDAR_BLIND_L_END   105.0f
#define LIDAR_BLIND_R_START 255.0f
#define LIDAR_BLIND_R_END   285.0f
```

Appliqué dans `robot.cpp` (détection obstacle) et `leds.cpp` (visualisation secteurs).
Calibrer en observant les points des poteaux sur Teleplot pendant WAIT_INIT.

---

## Affichage OLED

| État | Affichage |
|---|---|
| `APPUYER INIT` | Équipe en grand + encodeurs (R/L mm) + LIDAR |
| `INSERER TIRETTE` | Équipe en grand + LIDAR |
| `PRET !` | Équipe en grand + LIDAR |
| Match | État + X/Y/Cap + temps restant + CPU |
| `OBSTACLE!` | Distance + angle + secteur |
| `REPLI !` | Endgame en cours |
| `MATCH FINI` | Fin de match |

---

## LEDs NeoPixel

```
      AVANT
  [4]  [5]  [6]
       [0]          ← couleur d'équipe
  [3]  [2]  [1]
      ARRIÈRE
```

- **LED 0** : couleur d'équipe
- **LEDs 1–6** : secteurs LIDAR — rouge = obstacle, vert dim = libre
- Seuil = `OBS_DETECT_DIST_MM`, rafraîchissement 100ms

---

## Paramètres de calibration clés

| Paramètre | Effet |
|---|---|
| `DRIVE_WHEEL_DIAM_MM` | Distances parcourues |
| `WHEELBASE_MM` | Angles de rotation |
| `ENC_WHEEL_DIAM_MM` | Distances mesurées par les roues codeuses |
| `ENC_WHEELBASE_MM` | Angles calculés depuis les roues codeuses |
| `LIDAR_OFFSET_DEG` | Orientation secteurs LIDAR |
| `LIDAR_BODY_DIST_MM` | Filtre intérieur robot (80mm) |
| `LIDAR_BLIND_L/R_*` | Zones aveugles poteaux structurels |
| `OBS_DETECT_DIST_MM` | Distance détection adversaire |
| `MATCH_DURATION_MS` | Durée totale match |
| `MATCH_ENDGAME_MS` | Déclenchement repli |
| `STOCK_TOOL_OFFSET_MM` | Offset centre robot→stock en prise |
| `STOCK_DEPOSE_OFFSET_MM` | Offset centre robot→stock en dépose |

---

## Architecture des tâches FreeRTOS

| Tâche | Cœur | Priorité | Rôle |
|---|---|---|---|
| `lidar` | 0 | 2 | Lecture UART + parsing LD06 |
| `encoders` | 0 | 3 | update() encodeurs 200Hz — gestion rollover PCNT |
| `strategy` | 1 | 2 | Pré-match + match + chrono + actionneurs |
| `motion` | 1 | 3 | **taskMotionControl 50 Hz** — PID continu, exécute Target |
| `display` | 1 | 1 | Rafraîchissement OLED 2 Hz |
| `LogServer` | 1 | 1 | Broadcast WebSocket (pose, lidar, motion, etc.) |
| `loop()` | 1 | — | LEDs 100ms · WiFi telemetry 200ms · Field click poll · Log match 500ms |

---

## Interface Web (data/index.html)

Le robot expose une interface Web accessible via WiFi (mode AP "Karibous" ou STA selon `WIFI_USE_STA` dans `config.h`). Adresse : `http://192.168.4.1` (AP) ou IP affichée sur l'OLED (STA).

### Layout

- **Terrain central** (canvas 900×600) : pose robot, marqueur cible MotionController, points LIDAR optionnels, POIs
- **Pose bar** sous le terrain : X / Y / θ / état / Nav
- **Panneau ROBOT** : 7 boutons de contrôle d'état
- **Onglets droite** : MOTION (debug + radar) / ACTIONNEURS (servos) / CALIBRATION (live) / LOGS

### Click + drag goto

Cliquer sur le terrain envoie un goto au robot. Cliquer-glisser permet de spécifier l'orientation finale (flèche rouge de preview, label θ° en temps réel).

**Autorisé uniquement** : après init et hors match en cours. Le curseur devient crosshair quand autorisé.

### Boutons ROBOT

| Bouton | Action |
|---|---|
| ⏹ Stop moteurs | Désactive les moteurs (toujours dispo) |
| ⚡ Activer moteurs | Réactive |
| ↩ WAIT_INIT | Retour boucle pré-match sans reset |
| ▶ Lancer Init | Rejoue runInitYellow/Blue |
| ▶ Lancer Match | Démarre le match (skip tirette) |
| ⏱ Stop match | Force la fin du match |
| ↺ Relancer Match | Reset + relance la stratégie |

### Calibration live

Onglet CALIBRATION : modifier les paramètres (PID, géométrie, vitesses) en RAM sans recompiler. Une fois validés sur le robot, **les copier dans `config.h`** pour la persistance — sinon perdus au reboot.

### Synchronisation POI

`data/poi.js` est auto-généré depuis `src/strategy/poi.h` par `scripts/sync_poi.py` à chaque build PIO. Ajouter un POI : éditer **uniquement** `poi.h`, puis ajouter une entrée dans `POI_META` (`index.html`) pour le label/couleur d'affichage.
