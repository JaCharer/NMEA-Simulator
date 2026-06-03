#ifdef SIMMULATE_NMEA2000
#pragma once

#define ESP32_CAN_TX_PIN GPIO_NUM_25  
#define ESP32_CAN_RX_PIN GPIO_NUM_26  

#include <NMEA2000.h>
#include <N2kMessages.h>
#include "config.h"


void setupN2K();
void sendN2KNavigation(const YachtState &snap);
void sendN2KWind(const YachtState &snap);
void sendN2KDepth(const YachtState &snap);
void sendN2KEngine(const YachtState &snap);
void runN2KUpdate();

#endif
