#include "LCD_Buttons.h"
#include <WiFi.h>

#ifdef USE_LCD
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprSOG = TFT_eSprite(&tft);
TFT_eSprite sprCOG = TFT_eSprite(&tft);
TFT_eSprite sprInfo = TFT_eSprite(&tft);
TFT_eSprite sprStat = TFT_eSprite(&tft);
TaskHandle_t GUITask;

void setupLCDButtons() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    tft.init();
    tft.setRotation(0);
    sprSOG.createSprite(100, 30);
    sprCOG.createSprite(100, 30);
    sprInfo.createSprite(130, 32);
    sprStat.createSprite(130, 12);

#ifdef USE_BUTTONS
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT);
#endif

    xTaskCreatePinnedToCore(guiTaskCode, "GUITask", 12288, NULL, 1, &GUITask, 0);
}
#endif
// =========================================================================
// EKRAN TFT i GUI
// =========================================================================

#ifdef USE_LCD
void initDisplayStatic()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(2, 0);

  if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)
  {
    tft.print(" IP: ");
    tft.println(WiFi.softAPIP());
  }
  else
  {
    tft.print(" IP: ");
    tft.println(WiFi.localIP());
  }
  tft.drawFastHLine(0, 12, 135, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(2, 62);
  tft.print("KNOTS SOG");
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(2, 117);
  tft.print("DEGREE COG");
}

void updateDisplay()
{
  float l_sog, l_cog, l_depth, l_wind;
  bool l_nav, l_udp, l_tcp;
  int l_log;

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
  {
    l_sog = yacht.sog;
    l_cog = yacht.cog;
    l_depth = yacht.depth;
    l_wind = yacht.tws;
    l_nav = yacht.navMode;
    l_udp = config.enableUDP;
    l_tcp = config.enableTCP;
    l_log = config.log_level;
    xSemaphoreGive(dataMutex);
  }
  else
  {
    if (config.log_level >= 1)
      Serial.println("[LCD] Timeout mutex - pomijam updateDisplay");
    return;
  }

  sprSOG.fillSprite(TFT_BLACK);
  sprSOG.setTextSize(4);
  sprSOG.setTextColor(TFT_WHITE, TFT_BLACK);
  sprSOG.setCursor(0, 0);
  sprSOG.printf("%.1f", l_sog);
  sprSOG.pushSprite(2, 30);

  sprCOG.fillSprite(TFT_BLACK);
  sprCOG.setTextSize(4);
  sprCOG.setTextColor(TFT_ORANGE, TFT_BLACK);
  sprCOG.setCursor(0, 0);
  sprCOG.printf("%03.0f", l_cog);
  sprCOG.pushSprite(2, 85);

  sprInfo.fillSprite(TFT_BLACK);
  sprInfo.setTextSize(2);
  sprInfo.setTextColor(TFT_CYAN, TFT_BLACK);
  sprInfo.setCursor(0, 0);
  sprInfo.printf("DEP:%.1f m", l_depth);
  sprInfo.setCursor(0, 16);
  sprInfo.printf("TWS:%.1f kn", l_wind);
  sprInfo.pushSprite(2, 135);

  sprStat.fillSprite(TFT_BLACK);
  sprStat.setTextSize(1);
  sprStat.setTextColor(TFT_GREEN, TFT_BLACK);
  sprStat.setCursor(0, 0);
  sprStat.printf("U:%d T:%d L:%d", l_udp, l_tcp, l_log);
  sprStat.setCursor(80, 0);
  sprStat.setTextColor(l_nav ? TFT_MAGENTA : TFT_DARKGREY, TFT_BLACK);
  sprStat.print(l_nav ? "NAVI" : "MAN.");
  sprStat.pushSprite(2, 175);
}
#endif

#if defined(USE_LCD) && defined(USE_BUTTONS)
void guiTaskCode(void *pvParameters)
{
  initDisplayStatic();
  updateDisplay();
  bool lastL = HIGH, lastR = HIGH;
  unsigned long lastDispUpdate = 0, debounceL = 0, debounceR = 0;
  unsigned long bothPressedTime = 0;
  int lastCountdownSec = -1; // Zmienna zapobiegająca migotaniu ekranu

  for (;;)
  {
    bool currentL = digitalRead(BTN_LEFT);
    bool currentR = digitalRead(BTN_RIGHT);

    // === SPRAWDZANIE RESETU FABRYCZNEGO (Oba przyciski wciśnięte) ===
    if (currentL == LOW && currentR == LOW)
    {
      if (bothPressedTime == 0)
      {
        bothPressedTime = millis(); // Start odliczania
        lastCountdownSec = 5;

        // Rysowanie początkowego ekranu ostrzegawczego
        tft.fillScreen(TFT_RED);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(10, 40);
        tft.println("RESET ZA:");
        tft.setTextSize(4);
        tft.setCursor(55, 80);
        tft.println(lastCountdownSec);
      }
      else
      {
        unsigned long elapsed = millis() - bothPressedTime;
        if (elapsed > 5000)
        {
          // Minęło 5 sekund - TWARDY RESET
          Serial.println("\n[RESET] Wykryto twardy reset! Przywracanie ustawien fabrycznych...");
          // Wyświetlenie komunikatu o resecie
          tft.fillScreen(TFT_RED);
          tft.setTextSize(2);
          tft.setCursor(10, 50);
          tft.println("FACTORY");
          tft.setCursor(10, 80);
          tft.println("RESET...");
          //  Przywracanie ustawień fabrycznych
          preferences.clear();
          delay(2000);
          ESP.restart();
        }
        else
        {
          // Trwa odliczanie
          int secLeft = 5 - (elapsed / 1000);
          // Odśwież cyfrę tylko, jeśli się zmieniła
          if (secLeft != lastCountdownSec && secLeft > 0)
          {
            lastCountdownSec = secLeft;
            tft.fillRect(0, 70, 135, 60, TFT_RED); // Czyścimy tylko obszar starej cyfry
            tft.setCursor(55, 80);
            tft.println(lastCountdownSec);
          }
        }
      }
    }
    else
    {
      // Jeśli użytkownik puści przyciski przed czasem (przerwie reset)
      if (bothPressedTime != 0)
      {
        bothPressedTime = 0;
        lastCountdownSec = -1;

        // Szybkie przywrócenie normalnego interfejsu
        initDisplayStatic();
        updateDisplay();
      }
    }
// === STANDARDOWA OBSŁUGA ZMIANY KURSU ===
    if (bothPressedTime == 0)
    {
      if (currentL == LOW && lastL == HIGH && (millis() - debounceL > 150))
      {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(80)))
        {
          yacht.navMode = false;
          yacht.cog -= 10.0;
          if (yacht.cog < 0.0) yacht.cog += 360.0;
          yacht.skipperActive = false;
          xSemaphoreGive(dataMutex);
        }
        updateDisplay();
        debounceL = millis();
      }

      if (currentR == LOW && lastR == HIGH && (millis() - debounceR > 150))
      {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(80)))
        {
          yacht.navMode = false;
          yacht.cog += 10.0;
          if (yacht.cog >= 360.0) yacht.cog -= 360.0;
          yacht.skipperActive = false;
          xSemaphoreGive(dataMutex);
        }
        updateDisplay();
        debounceR = millis();
      }
    }

    lastL = currentL;
    lastR = currentR;

    // Odświeżanie danych na ekranie co 500ms
    if (bothPressedTime == 0 && millis() - lastDispUpdate > 500)
    {
      updateDisplay();
      lastDispUpdate = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
#endif

