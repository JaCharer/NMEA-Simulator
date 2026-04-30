#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <sys/time.h>
#include <time.h>
#include "config.h"

bool checkCollisionRisk_ARPA(float lat1, float lon1, float sog1, float cog1,
                             float lat2, float lon2, float sog2, float cog2);