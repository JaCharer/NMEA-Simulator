#pragma once
#include <Arduino.h>
#include "config.h" // Potrzebne dla struktury YachtState

// Udostępniamy uchwyt, gdybyśmy chcieli monitorować zadanie z main.cpp
extern TaskHandle_t SignalKTaskHandle;

// Prototypy
void signalKTaskCode(void *pvParameters);
void sendSignalKData(const YachtState &snap);