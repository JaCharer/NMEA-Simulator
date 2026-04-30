// =========================================================================
// SYMULATOR JACHTU NMEA / AIS
// Wersja: 3.2 NMEA + AIS + Signal K + Skipper AI
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
//#ifdef USE_LCD
//#include <TFT_eSPI.h>
//#endif
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

#include "config.h"
#include "ais.h"
#include "runSkipperAI.h"
#include "yacht_physics.h"
#include "LCD_Buttons.h"
#include "ARPA.h"
#include "network.h"
#include "WebServer.h"
#include "SignalK.h"
#include "NMEA0183.h"


// --- DEFINICJE ZMIENNYCH GLOBALNYCH ---
// To tutaj fizycznie powstają obiekty, do których odwołują się inne pliki przez extern
YachtState yacht;
SystemConfig config;
SemaphoreHandle_t dataMutex;


// --- ZMIENNE GLOBALNE I OBIEKTY ---
// --- KONFIGURACJA NTP ---
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;     // UTC = 0
const int daylightOffset_sec = 0; // W UTC nie ma czasu letniego

Preferences preferences;


// --- KONFIGURACJA WIFI ---
const char *ap_ssid_default = "Jacht_Symulator";
const char *ap_password = "password123";



// =========================================================================
// SETUP
// =========================================================================
void setup()
{
  Serial.begin(921600);
  randomSeed(esp_random());

  // <<< CZAS AWARYJNY USTAWIANY ZAWSZE NA STARCIE >>>
  struct timeval tv;
  tv.tv_sec = 1767268800; // 2026-01-01 12:00:00 UTC
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  Serial.println("[SYSTEM] Ustawiono startowy czas awaryjny: 2026-01-01 12:00:00 UTC");

  // --- INICJALIZACJA SYSTEMU PLIKÓW (LittleFS) ---
  if (!LittleFS.begin(true))
  {
    Serial.println("[BŁĄD] Wystapil problem podczas montowania systemu LittleFS");
    // Mimo błędu symulator może działać, ale strony WWW się nie załadują.
  }
  else
  {
    Serial.println("[SYSTEM] LittleFS zamontowany poprawnie.");
  }

  // --- INICJALIZACJA MUTEXA DANYCH ---
  dataMutex = xSemaphoreCreateMutex();

  // --- INICJALIZACJA TFT I KLAWISZY ---
#ifdef USE_LCD
  setupLCDButtons();
#endif
  // preferences.clear();   // odkomentuj tylko raz, żeby wyczyścić stare dane

  preferences.begin("jacht-config", false);

  // Wczytanie konfiguracji WiFi i ustawień
  config.wifi_ssid = preferences.getString("wifi_ssid", "");
  config.wifi_pass = preferences.getString("wifi_pass", "");
  config.useAP_mode = preferences.getBool("useAP", false); // Tryb AP
  config.enableUDP = preferences.getBool("udp", true);     // Włącz UDP
  config.enableTCP = preferences.getBool("tcp", true);     // Włącz TCP
  config.log_level = preferences.getInt("dbg", 1);         // Poziom logowania
  config.enableSignalK = preferences.getBool("sk", false); // Włącz SignalK

  Serial.println("\n=== KONFIGURACJA WCZYTANA ===");
  Serial.printf("SSID      : '%s'\n", config.wifi_ssid.c_str());
  Serial.printf("AP mode   : %s\n", config.useAP_mode ? "TAK" : "NIE");
  Serial.printf("Log Level : %d\n", config.log_level);
  Serial.printf("SignalK   : %s\n", config.enableSignalK ? "TAK" : "NIE");

  yacht.cog = (float)random(0, 360);
  yacht.target_nav_cog = yacht.cog;
  yacht.heading = yacht.cog;
  yacht.sog = 0.01f; // Minimalny ruch, który "budzi" ikony w OpenCPN

#ifdef STEADY_WIND_FOR_TESTS
  yacht.target_tws = STEADY_WIND_SPEED;
  yacht.target_twd = STEADY_WIND_DIR;
  yacht.tws = STEADY_WIND_SPEED;
  yacht.twd = STEADY_WIND_DIR;
#else
  yacht.target_tws = random(40, 270) / 10.0;
  yacht.target_twd = random(0, 360);
  yacht.tws = yacht.target_tws;
  yacht.twd = yacht.target_twd;
#endif

  // --- LOGIKA POŁĄCZENIA WIFI ---
  if (config.useAP_mode || config.wifi_ssid.length() == 0)
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid_default, ap_password);
    if (config.log_level >= 1)
    {
      Serial.println("\n[SIEC] Tryb AP uruchomiony z wyboru lub braku SSID.");
    }
    Serial.print("[SIEC] IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    WiFi.mode(WIFI_STA);
    bool connected = false;

    // Pętla 3 prób połączenia
    for (int attempt = 1; attempt <= 3; attempt++)
    {
      if (config.log_level >= 1)
      {
        Serial.printf("\n[SIEC] Łączenie z siecią: %s (Próba %d/3)\n", config.wifi_ssid.c_str(), attempt);
      }

      WiFi.disconnect(); // Rozłącz przed nową próbą, resetuje stan modułu
      delay(100);
      WiFi.begin(config.wifi_ssid.c_str(), config.wifi_pass.c_str());
      WiFi.setSleep(false);

      unsigned long startAttempt = millis();
      // Czekaj maksymalnie 10 sekund na jedną próbę
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000)
      {
        delay(500);
        Serial.print(".");
      }

      if (WiFi.status() == WL_CONNECTED)
      {
        connected = true;
        break; // Połączono pomyślnie - przerywamy pętlę
      }
      else
      {
        Serial.println(" Nieudana.");
      }
    }

    // Jeśli po 3 próbach nadal nie ma połączenia - uruchom AP (Fallback)
    if (!connected)
    {
      Serial.println("\n[SIEC] Błąd połączenia po 3 próbach - uruchamiam tryb ratunkowy AP"); //
      WiFi.mode(WIFI_AP);                                                                     // Wymuszenie trybu AP
      WiFi.softAP(ap_ssid_default, ap_password);
      Serial.print("[SIEC] IP AP: ");
      Serial.println(WiFi.softAPIP());
    }
    else
    {
      Serial.print("\n[SIEC] Połączono! IP: ");
      Serial.println(WiFi.localIP());
      // Włącz automatyczne wznawianie połączenia na wypadek późniejszej utraty zasięgu
      WiFi.setAutoReconnect(true);
      // Synchronizacja czasu NTP (UTC)
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //
      if (config.log_level >= 1)
      {
        Serial.println("[SYSTEM] Uruchomiono synchronizację czasu NTP (UTC).");
      }
    }
  }

  udp.begin(port);
  nmeaServer.begin();
  setupWebServer();

  // --- Type 24A (pełne dane statyczne) ---
  generateType24A(261000001, targets[0].name, targets[0].staticNmeaA);
  generateType24A(261000002, targets[1].name, targets[1].staticNmeaA);
  generateType24A(261000003, targets[2].name, targets[2].staticNmeaA);
  // --- Type 24B (pełne dane statyczne) ---
  generateType24B(261000001, "PROM1", 60, 150.0, 25.0, targets[0].staticNmeaB);      // Passenger
  generateType24B(261000002, "TANK2", 80, 220.0, 32.0, targets[1].staticNmeaB); // Tanker
  generateType24B(261000003, "FISH3", 30, 22.0, 7.0, targets[2].staticNmeaB);       // Fishing

  esp_task_wdt_init(15, true); // 15 sekund timeout
  esp_task_wdt_add(NULL);      // dodaje bieżący task (loop)

  // --- NOWY TASK: SignalK (RDZEŃ 0, ten sam co WiFi) ---
  xTaskCreatePinnedToCore(
      signalKTaskCode,    /* Funkcja zadania */
      "SignalKTask",      /* Nazwa */
      8192,               /* Rozmiar stosu */
      NULL,               /* Parametry */
      1,                  /* Priorytet */
      &SignalKTaskHandle, /* Handle */
      0                   /* RDZEŃ 0 (Ten sam co WiFi) */
  );
}



// =========================================================================
// GŁÓWNA PĘTLA – TERAZ LEKKA I CZYTELNA
// =========================================================================
void loop()
{
  // --- SYSTEM I SPRZĄTANIE (Zawsze na Core 1) ---
  ws.cleanupClients();
  skWs.cleanupClients();
  esp_task_wdt_reset();

  // --- HARMONOGRAM ---
  static unsigned long lastTick = 0;
  static int tick = 0;
  static int packetCounter = 0;
  static char nmeaPack[NMEA_BUFFER_SIZE];
  static char aisPack[NMEA_BUFFER_SIZE];

  if (millis() - lastTick >= 100)
  {
    lastTick = millis();
    tick++;
    if (tick > 10)
    {
      tick = 1;
      packetCounter++;
    }

    // 1. WEJŚCIA I SIEĆ (Najpierw odbieramy polecenia, by fizyka mogła na nie zareagować)
    manageNetworkConnections();
    handleIncomingTcpData();

    // 2. SYMULACJA (Obliczamy fizykę uwzględniając nowe polecenia z kroku 1)
    updateYachtPhysics();

    // 3. SNAPSHOT (Zamrażamy stan jachtu - "Mózg" i "Sieć" pracują na identycznych danych)
    YachtState snap;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
    {
      snap = yacht;
      xSemaphoreGive(dataMutex);
    }

    // 4. PODZIAŁ PRACY (Switch-case rozkłada obciążenie w czasie)
    switch (tick)
    {
    case 1:
      if (snap.skipperActive)
      {
        runSkipperAI(snap);
      }
      break;

    case 3:
    {
      // Budowanie i wysyłka AIS
      memset(aisPack, 0, NMEA_BUFFER_SIZE);
      int aisOffset = 0;
      updateAisTargetsAndSentences(aisPack, aisOffset, snap);
      broadcastNmeaToNetwork(aisPack);
      break;
    }

    case 6:
    {
      // Budowanie NMEA (tylko budowanie!)
      memset(nmeaPack, 0, NMEA_BUFFER_SIZE);
      int offset = 0;
      time_t now;
      struct tm timeinfo;
      time(&now);
      gmtime_r(&now, &timeinfo);
      generateNmeaSentences(nmeaPack, offset, packetCounter, &timeinfo, snap);
      break;
    }

    case 8:
      // Wysyłka NMEA (200ms po zbudowaniu - rozłożenie obciążenia)
      if (nmeaPack[0] != '\0')
      {
        broadcastNmeaToNetwork(nmeaPack);
      }
      break;

    case 10:
      updateBrowserDashboard(snap);
      handleStaticAisBroadcasting();
      break;
    }
  }

  // To musi być poza blokiem 100ms, aby RTOS mógł przełączyć rdzenie
  vTaskDelay(pdMS_TO_TICKS(1));
}
// END OF FILE