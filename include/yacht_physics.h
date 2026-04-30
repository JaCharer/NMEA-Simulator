#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"

float calculateTargetSOG(float cog, float twd, float tws, bool eng_on, float rpm);
void calculateApparentWind(float cog, float sog, float twd, float tws, float &awa,
                           float &aws, float &twa);
void updateYachtPhysics();