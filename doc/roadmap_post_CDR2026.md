# Roadmap post-CDR2026

Cahier des charges des évolutions à implémenter après la Coupe de France de Robotique 2026.
Le code de référence compétition est archivé sur la branche `CDR2026`.

---

## Statuts

| Icône | Signification |
| --- | --- |
| ⬜ | À faire |
| 🔵 | En cours de spécification |
| 🟡 | En cours d'implémentation |
| ✅ | Terminé |

---

## 1. Architecture motion — Asservissement continu

### Contexte

Actuellement, chaque déplacement (`gotoXYenc`, `goPID`, etc.) est **bloquant** : la tâche stratégie attend la fin du mouvement avant de passer à la suivante. Cela entraîne :
- Un arrêt complet à chaque waypoint (perte de temps)
- Aucune correction de position à l'arrêt (seul le couple de maintien stepper résiste aux poussées)
- Impossibilité de chaîner les mouvements fluidement

### 1.1 ⬜ Tâche de contrôle moteur continue (`taskMotionControl`)

**Objectif** : Remplacer `_runWheelPID` bloquant par une tâche FreeRTOS permanente qui tourne à 50 Hz (ou 20 ms / `OBS_POLL_MS`).

**Principe** :
- La tâche lit en permanence la pose encodeur (`gDisplay.enc_pose_*`)
- Elle calcule l'erreur par rapport à une **cible courante** partagée
- Elle envoie les vitesses moteur via `setMotorVelocities`
- En l'absence de cible active : mode **HOLD** (cible = position courante)

**États de la tâche** :

```
IDLE   → moteurs libres (fin de match, init)
MOVING → PID actif, convergence vers cible
HOLD   → PID actif, cible = position courante (résistance aux poussées)
```

**Interface stratégie** :
```cpp
robot.setTarget(x, y, theta);   // non-bloquant, met à jour la cible
robot.waitArrived(blendMm);     // bloquant optionnel (attend avg < blendMm)
robot.holdPosition();           // force le mode HOLD
```

**Thread safety** : la cible (x, y, theta) est écrite par la stratégie (Core 1) et lue par `taskMotionControl`. Nécessite un struct atomique ou `portMUX_TYPE`.

**Tâche** : Core 1, priorité 3 (entre strategy prio 2 et display prio 1).

---

### 1.2 ⬜ Blend — Enchaînement sans arrêt entre waypoints

**Objectif** : Ne pas s'arrêter à chaque waypoint, transition fluide vers le suivant.

**Principe** :
- `setTarget(B)` peut être appelé alors que le robot est encore en route vers A
- Quand `avg <= blendMm`, la cible bascule vers B sans softStop
- `speedCap` initialisé à la **vitesse courante** (pas de ramp depuis 0)

**Règle de sécurité** (angle de virage) :
```
angle_virage < 30°  → blend autorisé, blendMm plein
angle_virage 30-60° → blendMm réduit (× cos(angle))
angle_virage > 60°  → arrêt complet forcé (blendMm = 0)
```

**Impact détection obstacle** : la direction `moveDir` est recalculée à chaque transition. Fenêtre aveugle ≈ `blendMm / vitesse` (< 150 ms à vitesse normale) — acceptable pour virages < 30°.

---

### 1.3 ⬜ Position hold — Résistance aux poussées

**Objectif** : Quand le robot est à l'arrêt, maintenir sa position via feedback encodeur (pas seulement via le couple de maintien stepper).

**Principe** : Mode HOLD de `taskMotionControl` — cible = position au moment du stop. Si poussée détectée (encodeur dérive), le PID commande une correction.

**Seuil** : si erreur < `HOLD_DEAD_BAND_MM` (ex: 2 mm), pas de commande moteur (évite l'échauffement par micro-corrections).

---

### 1.4 ⬜ API followPath

**Objectif** : Envoyer une liste de waypoints en une seule commande.

```cpp
robot.followPath({
    {POI::stockYellow_01, ANGLE_WEST,  150.0f},  // {poi, angle, blendMm}
    {POI::deposeZone,     ANGLE_NORTH, 100.0f},
    {POI::startYellow,    ANGLE_SOUTH,   0.0f},  // arrêt complet
}, speed);
```

La tâche motion consomme la file et enchaîne les cibles avec blend.

---

## 2. Interface web de debug

### Contexte

L'interface actuelle (`data/index.html`) affiche : position robot, scan LIDAR (radar), logs, actionneurs. Pour débugger l'asservissement continu et le blend, il manque plusieurs éléments.

---

### 2.1 ⬜ Marqueur de cible sur le terrain

**Objectif** : Afficher la **cible courante** sur la carte du terrain (en plus de la position robot).

- Symbole : croix verte `⊕` ou cercle vide
- Rayon de blend : cercle semi-transparent autour de la cible
- Mise à jour : nouveau message WebSocket `{"type":"target","x":...,"y":...,"blend":150}`

---

### 2.2 ⬜ File de waypoints sur le terrain

**Objectif** : Afficher les waypoints en attente sous forme de traits pointillés.

- Ligne pointillée entre robot → waypoint 1 → waypoint 2 → ...
- Message : `{"type":"path","pts":[[x1,y1],[x2,y2],...]}`
- Mise à jour à chaque `setTarget` ou `followPath`

---

### 2.3 ⬜ Points LIDAR sur le terrain (coordonnées absolues)

**Objectif** : Afficher le scan LIDAR projeté sur la carte du terrain (en plus du radar relatif robot).

- Transformation : angle LIDAR + distance + pose robot → coordonnées table (X, Y mm)
- Rendu : petits points colorés superposés à l'image du terrain
- Filtre : même filtre que la détection obstacle (confidence, body_dist, murs)
- Toggle show/hide dans l'UI (peut charger le rendu)
- Message : `{"type":"lidar_abs","pts":[[x1,y1],[x2,y2],...]}` (différent du radar relatif)

**Calcul côté ESP** :
```cpp
float a = (270.0f - angle_deg + LIDAR_OFFSET_DEG + robot_theta_deg) * DEG2RAD;
float wx = robot_x + dist * cosf(a);
float wy = robot_y - dist * sinf(a);  // Y+ vers le bas
```

---

### 2.4 ⬜ Panneau "Motion debug"

**Objectif** : Afficher les données de la tâche motion en temps réel.

Éléments :
- État : `IDLE` / `MOVING` / `HOLD` / `OBSTACLE`
- Distance à la cible (mm)
- Angle d'erreur vers la cible (°)
- `speedCap` courant (mm/s) — voir la rampe
- Vitesse commandée roue gauche / droite (mm/s)

Intégration dans la pose bar ou panneau dédié sous le radar.

Message WebSocket étendu :
```json
{
  "type": "pose",
  "x": 375, "y": 225, "a": 90,
  "state": "MOVING",
  "nd": 450, "na": 2.1,
  "vL": 1200, "vR": 1180,
  "cap": 980,
  "tm": "Y"
}
```

---

### 2.5 ⬜ Graphe temps réel des vitesses

**Objectif** : Courbes scrollantes `outL` / `outR` / `avg` pour calibrer le PID et observer le profil vitesse (rampe, freinage, blend).

- Canvas dédié (ex: 600 × 150 px) sous le panneau motion
- Buffer circulaire JS (dernières N secondes)
- Toggle show/hide (optionnel selon les besoins de debug)

---

## 3. Autres améliorations identifiées

### 3.1 ⬜ Synchronisation POI entre poi.h et index.html

**Contexte** : Les coordonnées des POIs sont dupliquées en C++ (`poi.h`) et en JS (`index.html`). Toute modification doit être faite aux deux endroits.

**Solution envisagée** : Générer `data/poi.js` depuis `poi.h` à la compilation (script Python dans `extra_scripts` de `platformio.ini`), inclus par `index.html`.

---

### 3.2 ⬜ Calibration live via interface web

**Contexte** : Les paramètres de calibration sont des `#define` dans `config.h` — chaque ajustement nécessite une recompilation. L'objectif est de permettre la modification en RAM sans recompiler, pour validation visuelle immédiate. Une fois validée, la valeur est reportée dans `config.h`.

**Principe** :

- Remplacement des `#define` ciblés par des variables globales dans `src/config_runtime.h/cpp`
- Handler WebSocket `{"type":"calib","key":"...","val":...}` dans `log_server`
- Panneau dédié dans l'UI avec sliders + affichage valeur courante
- **Pas de persistance NVS** — valeur en RAM uniquement, perdue au reboot (intentionnel)

**Paramètres groupe 1 — Calibration mécanique** (varient selon usure et montage) :

| Paramètre | Unité | Plage indicative |
| --- | --- | --- |
| `ENC_WHEEL_DIAM_MM` | mm | 45 – 55 |
| `ENC_WHEELBASE_MM` | mm | 170 – 210 |
| `WHEELBASE_MM` | mm | 130 – 160 |
| `DRIVE_WHEEL_DIAM_MM` | mm | 52 – 62 |
| `LIDAR_OFFSET_DEG` | ° | -10 – +10 |

**Paramètres groupe 2 — Réglage PID** (tuning, stables une fois trouvés) :

| Paramètre | Unité | Plage indicative |
| --- | --- | --- |
| `ENC_P1_KP` | mm/s par mm | 0.5 – 5.0 |
| `ENC_P1_KI` | mm/s par mm·s | 0 – 0.1 |
| `ENC_P1_KD` | — | 0 – 1.0 |
| `ENC_P1_MIN_SPD` | mm/s | 5 – 20 |
| `ENC_P1_STOP_MM` | mm | 1 – 10 |
| `DEFAULT_SPEED_MMS` | mm/s | 500 – 3000 |
| `DEFAULT_ACCEL_MMS2` | mm/s² | 500 – 3000 |

**Workflow cible** :

```text
Slider UI → WebSocket → g_param (RAM) → effet immédiat visible
→ valeur validée → reportée dans config.h → pio run -t upload
```

---

### 3.3 ⬜ Contrôle d'état du robot via interface web

**Contexte** : Actuellement, pour relancer une séquence (init, match…), il faut appuyer sur le bouton reset physique et recommencer le cycle complet. Cela ralentit les tests.

**Objectif** : Depuis l'interface web, pouvoir déclencher un retour à un état particulier sans reset matériel.

**États déclenchables depuis l'UI** :

| Bouton | Action |
| --- | --- |
| `⟳ Relancer Init` | Relance `runInitYellow/Blue` depuis la position courante — recalage bordures sans reboot |
| `▶ Relancer Match` | Relance `runStrategyYellow/Blue` depuis la position actuelle (sans recalage) |
| `⏹ Stop moteurs` | Appelle `disableMotors()` — moteurs libres |
| `⟲ Retour WAIT_INIT` | Remet le robot en attente d'init (reset état stratégie) |

**Principe d'implémentation** :

- Nouveau type de message WebSocket : `{"type":"state_cmd","action":"restart_init"}`
- La tâche stratégie expose une file de commandes (QueueHandle_t)
- Un handler dans `log_server` pousse la commande dans la file
- La tâche stratégie consomme la file à chaque boucle et exécute l'action

**Contraintes de sécurité** :

- Boutons grisés pendant le match (state = MOVING, GOTO, etc.)
- Confirmation requise pour "Relancer Match" (évite les fausses manœuvres)
- `Stop moteurs` toujours accessible (bouton d'urgence)

---

## Ordre de priorité suggéré

| Priorité | Item | Dépendances |
| --- | --- | --- |
| 1 | `taskMotionControl` (1.1) | Base de tout le reste |
| 2 | Marqueur cible + Motion debug (2.1, 2.4) | 1.1 |
| 3 | Contrôle d'état via UI (3.3) | Indépendant |
| 4 | Blend (1.2) | 1.1 |
| 5 | Points LIDAR sur terrain (2.3) | Indépendant |
| 6 | Calibration live groupe 1 — mécanique (3.2) | 2.3 (LIDAR) |
| 7 | Position hold (1.3) | 1.1 |
| 8 | Calibration live groupe 2 — PID (3.2) | 1.1 (motion debug) |
| 9 | Graphe temps réel (2.5) | 2.4 |
| 10 | followPath API (1.4) | 1.2 |
| 11 | File waypoints UI (2.2) | 1.4 |
| 12 | Sync POI (3.1) | Indépendant |
