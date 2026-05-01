#pragma once
#include "../config.h"
#include "../lidar/ld06.h"

void ledsInit();
void ledsSetTeam(Team t);
void ledsUpdateLidar(const LidarPoint *pts, uint16_t n);
