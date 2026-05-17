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

```text
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

```text
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

L'interface actuelle (`data/index.html`) est fonctionnelle. Plusieurs éléments restent à ajouter pour débugger l'asservissement continu et le blend.

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

### 2.3 ✅ Points LIDAR sur le terrain (coordonnées absolues)

Scan LIDAR projeté en coordonnées table, filtré aux limites (0→3000 × 0→2000 mm).
Toggle "● LIDAR terrain" dans la barre des filtres. Points bleus, cercles r=2.5px.
Message `{"type":"lidar_abs","pts":[[x,y],...]}` envoyé toutes les 100 ms.

---

### 2.4 ⬜ Panneau "Motion debug"

**Objectif** : Afficher les données de la tâche motion en temps réel.

Éléments :

- État : `IDLE` / `MOVING` / `HOLD` / `OBSTACLE`
- Distance à la cible (mm), angle d'erreur (°)
- `speedCap` courant (mm/s), vitesses L/R commandées

---

### 2.5 ⬜ Graphe temps réel des vitesses

**Objectif** : Courbes scrollantes `outL` / `outR` / `avg` pour calibrer le PID.

- Canvas dédié, buffer circulaire JS
- Toggle show/hide

---

## 3. Autres améliorations

### 3.1 ⬜ Synchronisation POI entre poi.h et index.html

**Contexte** : Coordonnées dupliquées en C++ et JS — tout changement doit être fait aux deux endroits.

**Solution** : Générer `data/poi.js` depuis `poi.h` à la compilation (script Python dans `extra_scripts`).

---

### 3.2 ⬜ Calibration live via interface web

Modification des paramètres de calibration en RAM sans recompilation. Valeur reportée dans `config.h` une fois validée.

**Groupe 1 — Calibration mécanique** :

| Paramètre | Unité | Plage |
| --- | --- | --- |
| `ENC_WHEEL_DIAM_MM` | mm | 45 – 55 |
| `ENC_WHEELBASE_MM` | mm | 170 – 210 |
| `WHEELBASE_MM` | mm | 130 – 160 |
| `DRIVE_WHEEL_DIAM_MM` | mm | 52 – 62 |
| `LIDAR_OFFSET_DEG` | ° | -10 – +10 |

**Groupe 2 — PID** :

| Paramètre | Unité | Plage |
| --- | --- | --- |
| `ENC_P1_KP` | mm/s/mm | 0.5 – 5.0 |
| `ENC_P1_KI` | — | 0 – 0.1 |
| `ENC_P1_KD` | — | 0 – 1.0 |
| `ENC_P1_MIN_SPD` | mm/s | 5 – 20 |
| `ENC_P1_STOP_MM` | mm | 1 – 10 |
| `DEFAULT_SPEED_MMS` | mm/s | 500 – 3000 |
| `DEFAULT_ACCEL_MMS2` | mm/s² | 500 – 3000 |

---

### 3.3 ✅ Contrôle d'état du robot via interface web

Panneau "ROBOT" dans l'UI avec 6 boutons :

| Bouton | Action |
| --- | --- |
| ⏹ Stop moteurs | `disableMotors()` — toujours accessible |
| ⚡ Activer moteurs | `enableMotors()` |
| ⏱ Stop match | Force `isMatchOver()` → true (confirmation) |
| ↩ WAIT_INIT | Retour boucle pré-match sans reset |
| ↺ Relancer Init | Rejoue `runInitXxx` (confirmation) |
| ▶ Relancer Match | Relance stratégie (confirmation) |

Implémentation : `StateCmd` enum dans `src/state_cmd.h`, `QueueHandle_t gStateCmd` consommée par `taskStrategy`, pushée par `log_server`.

---

## Ordre de priorité — Restant

| Priorité | Item | Dépendances |
| --- | --- | --- |
| 1 | `taskMotionControl` (1.1) | Base de tout le reste |
| 2 | Marqueur cible + Motion debug (2.1, 2.4) | 1.1 |
| 3 | Blend (1.2) | 1.1 |
| 4 | Position hold (1.3) | 1.1 |
| 5 | Calibration live groupes 1+2 (3.2) | Indépendant |
| 6 | Graphe temps réel (2.5) | 2.4 |
| 7 | followPath API (1.4) | 1.2 |
| 8 | File waypoints UI (2.2) | 1.4 |
| 9 | Sync POI (3.1) | Indépendant |
