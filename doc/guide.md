# Guide d'utilisation — Differential Robot Firmware

## Vue d'ensemble

Firmware pour robot différentiel de compétition (Coupe de France de Robotique) basé sur un **ESP32-S3**. Il gère la motorisation pas-à-pas, le LIDAR LD06, la détection d'adversaire, l'affichage OLED et les LEDs NeoPixel.

---

## Matériel requis

| Composant | Détail |
|---|---|
| Microcontrôleur | ESP32-S3-DevKitC-1 |
| Moteurs | 2× pas-à-pas (200 pas/tr, 8 micropas) |
| LIDAR | LD06 |
| Écran | OLED SSD1306 128×64 I2C |
| LEDs | NeoPixel × 7 (pin 46) |
| Tirette | Pin 14, pull-up externe |
| Switch équipe | Pin 17, pull-up interne |
| Bouton init | Pin 3, pull-up interne |

---

## Démarrage rapide

### 1. Configurer le matériel — `src/config.h`

C'est **le seul fichier à toucher** pour adapter le firmware à ton robot.

```c
// Diamètre réel des roues (mesurer avec un pied à coulisse)
#define DRIVE_WHEEL_DIAM_MM   57.7f

// Distance entre les deux roues motrices
#define WHEELBASE_MM          148.5f

// Décalage arrière du robot → axe des roues (pour le recalage bordure)
#define ROBOT_BACK_TO_CENTER_MM  80.9f

// Vitesse et accélération par défaut
#define DEFAULT_SPEED_MMS     200.0f   // mm/s
#define DEFAULT_ACCEL_MMS2    150.0f   // mm/s²

// Distance de détection adversaire
#define OBS_DETECT_DIST_MM    400.0f   // mm
```

### 2. Flasher et démarrer

```bash
pio run --target upload
pio device monitor
```

---

## Séquence pré-match

À chaque démarrage, le robot suit cette séquence avant de lancer la stratégie :

```
1. APPUYER INIT   → positionner le robot contre la bordure, appuyer sur le bouton INIT
                    → le robot effectue le calage et mémorise sa position

2. INSERER TIRETTE → insérer la tirette (pin 14 passe LOW)

3. PRET !          → le robot attend le retrait de la tirette

4. [Tirette retirée] → match lancé !
```

Le **switch d'équipe** (pin 17) peut être changé à tout moment avant l'étape 1.
Si l'équipe change après l'init, l'init est automatiquement réinitialisée.

---

## Écrire une stratégie — `src/strategy/strategy.cpp`

### Fonctions disponibles

```cpp
// ─── Déplacements ───────────────────────────────────────────────────────────
robot.go(500);              // avance 500 mm (négatif = recule)
robot.turn(90);             // tourne 90° à gauche (négatif = droite)
robot.gotoXY(1000, 500);    // va aux coordonnées absolues (1000, 500)
robot.gotoXY(1000, 500, 0); // idem avec cap final 0° (face à X+)

// ─── Pose ───────────────────────────────────────────────────────────────────
robot.setPosition(x, y, theta_deg);  // recalage / position initiale

// ─── Vitesse ────────────────────────────────────────────────────────────────
robot.setSpeed(400);        // change la vitesse en mm/s

// ─── Détection adversaire ───────────────────────────────────────────────────
robot.enableObstacle();     // active la détection (par défaut : off)
robot.disableObstacle();    // désactive

// ─── Moteurs ────────────────────────────────────────────────────────────────
robot.disableMotors();      // désactive les moteurs (robot poussable)
robot.enableMotors();       // réactive les moteurs
```

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
- **X+** : vers la droite (3000 mm)
- **Y+** : vers le bas (2000 mm)
- **Angle 0°** : face à droite (+X)
- **Angles positifs** : sens anti-horaire — 90° = haut (-Y), 270° = bas (+Y)
- **`turn(+90)`** : tourne à gauche (CCW) — `turn(-90)` : tourne à droite (CW)

### Exemple de stratégie jaune

```cpp
void runStrategyYellow(Robot &robot) {
    robot.enableObstacle();
    robot.setPosition(200, 1000, 0);  // position après calage bordure

    robot.go(500);          // avance 500 mm
    robot.turn(-90);        // tourne à droite
    robot.go(300);          // avance 300 mm
    robot.gotoXY(800, 500); // va en (800, 500)
}
```

### Init — recalage bordure

```cpp
void runInitYellow(Robot &robot) {
    robot.disableObstacle();
    robot.go(-500);         // recule contre la bordure

    // La position X = TABLE_MARGIN + ROBOT_BACK_TO_CENTER_MM
    robot.setPosition(300, ROBOT_BACK_TO_CENTER_MM, 0);
}
```

---

## Affichage OLED

| État | Affichage |
|---|---|
| Pré-match | État + nom d'équipe en grand + statut LIDAR |
| Match | État + X/Y/Cap + CPU |
| OBSTACLE | Distance + angle + secteur (AVANT GAUCHE, etc.) |

---

## LEDs NeoPixel

```
      AVANT
  [4]  [5]  [6]
       [0]          ← couleur d'équipe
  [3]  [2]  [1]
      ARRIÈRE
```

- **LED 0** : couleur d'équipe (jaune ou bleu)
- **LEDs 1–6** : secteurs LIDAR — rouge = obstacle, vert dim = libre
- Seuil : `OBS_DETECT_DIST_MM` (400 mm par défaut)

---

## Détection d'adversaire

La détection est active uniquement pendant `go()` (pas pendant `turn()`).

- **Avant** : si le robot avance
- **Arrière** : si le robot recule

Comportement sur détection :
1. Freinage avec `OBS_STOP_ACCEL_MMS2` = 2000 mm/s²
2. Recul de `OBS_BACKUP_MM` = 100 mm
3. Attente que l'adversaire dégage (max `OBS_WAIT_MS` = 3 s)
4. Reprise du mouvement restant automatiquement

```cpp
// Paramètres ajustables dans config.h
#define OBS_DETECT_DIST_MM   400.0f   // distance de détection
#define OBS_WIDTH_MM         200.0f   // largeur de la zone (≥ largeur robot)
#define OBS_BACKUP_MM        100.0f   // recul après détection
#define OBS_STOP_ACCEL_MMS2 2000.0f   // décélération d'urgence
#define OBS_WAIT_MS          3000     // timeout adversaire (ms)
```

---

## Paramètres de calibration clés

| Paramètre | Effet | À ajuster si... |
|---|---|---|
| `DRIVE_WHEEL_DIAM_MM` | Distances parcourues | Le robot fait plus/moins que la distance demandée |
| `WHEELBASE_MM` | Angles de rotation | Le robot sur/sous-tourne |
| `LIDAR_OFFSET_DEG` | Orientation secteurs LIDAR | Les secteurs LED sont décalés |
| `LIDAR_BODY_DIST_MM` | Filtre intérieur robot | Faux positifs à courte distance |
| `OBS_DETECT_DIST_MM` | Distance de détection | Trop/pas assez réactif |

---

## Architecture des tâches FreeRTOS

| Tâche | Cœur | Priorité | Rôle |
|---|---|---|---|
| `lidar` | 0 | 2 | Lecture UART + parsing LD06 |
| `strategy` | 1 | 2 | Pré-match + match |
| `display` | 1 | 1 | Rafraîchissement OLED 2 Hz |
| `loop()` | 1 | — | LEDs 100 ms + Teleplot 500 ms |
