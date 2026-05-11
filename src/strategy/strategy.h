#pragma once
#include "robot.h"
#include "../config.h"
#include "../motion/encoder.h"

// ─── Calibration géométrique ─────────────────────────────────────────────────
// Effectue go(1000) + turn(360), mesure les encodeurs et affiche les nouvelles
// valeurs DRIVE_WHEEL_DIAM_MM et WHEELBASE_MM sur le moniteur série.
void runCalibration(Robot &robot, QuadEncoder &encL, QuadEncoder &encR);

// ─── Calage bordure (à appeler avant la mise en place sur table) ─────────────
void runInitYellow(Robot &robot);
void runInitBlue(Robot &robot);

// ─── Prise / dépose de stock ─────────────────────────────────────────────────
// x, y       : centre du stock (mm, repère table)
// angleDeg   : orientation d'approche (0°=droite, 90°=haut/-Y, etc.)
// takeStock  : offset prise  = STOCK_TOOL_OFFSET_MM  (210 mm)
// deposeStock: offset dépose = STOCK_DEPOSE_OFFSET_MM (180 mm)
void takeStock (Robot &robot, float x, float y, float angleDeg);
void takeStock (Robot &robot, Vec2 poi,   float angleDeg);
void deposeStock(Robot &robot, float x, float y, float angleDeg);
void deposeStock(Robot &robot, Vec2 poi,   float angleDeg);

// ─── Stratégie de match ───────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot);
void runStrategyBlue(Robot &robot);

// ─── Repli fin de match (déclenché à MATCH_ENDGAME_MS) ───────────────────────
void runNearEndYellow(Robot &robot);
void runNearEndBlue(Robot &robot);

