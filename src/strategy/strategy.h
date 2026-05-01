#pragma once
#include "robot.h"
#include "../config.h"

// ─── Calage bordure (à appeler avant la mise en place sur table) ─────────────
void runInitYellow(Robot &robot);
void runInitBlue(Robot &robot);

// ─── Stratégie de match ───────────────────────────────────────────────────────
void runStrategyYellow(Robot &robot);
void runStrategyBlue(Robot &robot);

// ─── Tests ───────────────────────────────────────────────────────────────────
void runTestObstacle(Robot &robot);
