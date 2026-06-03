#pragma once
#include <Arduino.h>
#include "config.h"

#ifdef USE_LCD
#include <TFT_eSPI.h>

// --- KONFIGURACJA PINÓW ---
#define BTN_LEFT 0
#define BTN_RIGHT 35
#define TFT_BL 4

extern TFT_eSPI tft;
extern TFT_eSprite sprSOG;
extern TFT_eSprite sprCOG;
extern TFT_eSprite sprHeader;
extern TFT_eSprite sprInfo;
extern TFT_eSprite sprStat;


void setupLCDButtons();
void initDisplayStatic();
bool updateDisplay();
void guiTaskCode(void *pvParameters);
extern TaskHandle_t GUITask;
#endif
