#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

void addNMEAChecksumToLine(const char *baseLine, char *outputLine);
void generateNmeaSentences(char *pack, int &offset, int packetCounter, struct tm *timeinfo, const YachtState &snap);