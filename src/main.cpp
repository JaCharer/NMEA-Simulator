// =========================================================================
// SYMULATOR MOSTKA KAPITAŃSKIEGO NMEA / AIS
// Wersja: 2.1 (Pełna konfiguracja WiFi + poprawiony RPM dla OpenCPN)
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
//#include <WebServer.h>
#include <Preferences.h>
#include <TFT_eSPI.h> 
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <sys/time.h>
#include <LittleFS.h>

// Funkcja pomocnicza do pobierania timestampu
String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return String("[NO TIME]");
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "[%04d-%02d-%02d %02d:%02d:%02d]",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return String(buf);
}

// --- KONFIGURACJA SPRZĘTU ---
#define BTN_LEFT  0
#define BTN_RIGHT 35
#define TFT_BL    4
//#define TOUCH_CS  -1 // TTGO T-Display nie ma dedykowanego pinu CS dla ekranu dotykowego, więc ustawiamy na -1
#define MAX_TCP_CLIENTS 3
WiFiClient tcpClients[MAX_TCP_CLIENTS];

// --- ZMIENNE GLOBALNE I OBIEKTY ---
char staticAIS_PromA[100];
char staticAIS_TankowiecA[100];
char staticAIS_RybakA[100];
unsigned long lastStaticAISTick = 0;
char staticAIS_PromB[100];
char staticAIS_TankowiecB[100];
char staticAIS_RybakB[100];

// --- KONFIGURACJA NTP ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;      // UTC = 0
const int   daylightOffset_sec = 0; // W UTC nie ma czasu letniego


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprSOG = TFT_eSprite(&tft);
TFT_eSprite sprCOG = TFT_eSprite(&tft);
TFT_eSprite sprInfo = TFT_eSprite(&tft);
TFT_eSprite sprStat = TFT_eSprite(&tft);

Preferences preferences;
TaskHandle_t GUITask;
SemaphoreHandle_t dataMutex;

// --- KONFIGURACJA WIFI ---
String wifi_ssid = "";
String wifi_pass = "";
bool   useAP_mode = false;

const char* ap_ssid_default = "Jacht_Symulator";
const char* ap_password = "password123";

//WebServer webServer(80); 
AsyncWebServer webServer(80);
AsyncWebSocket ws("/ws");

const int port = 10110; 
WiFiUDP udp;
const char* udpAddress = "255.255.255.255"; 
WiFiServer nmeaServer(port); 
WiFiClient tcpClient;    

// --- STRUKTURA FLOTY (Cele AIS) – realistyczne zachowanie ---
// --- STRUKTURA FLOTY (Cele AIS) ---
struct Target {
  int id;
  float lat, lon, cog, sog; // Podstawowe dane pozycyjne i ruchu
  const char* name;         // Nazwa statku (dla AIS)
  
  float max_radius_nm;      // Promień strefy (smycz)
  float base_sog;           // Nominalna prędkość statku
  float max_sog;            // Prędkość pościgowa (efekt gumki)
  bool is_returning;        // Flaga dla Rybaka
  
  float target_cog;         // Kurs, na który aktualnie skręca statek
  float planned_cog;        // Kurs, do którego statek wróci po manewrze
  bool is_avoiding;         // Flaga: Czy statek jest w trakcie unikania kolizji
  
  unsigned long lastCourseDecision;  
  float max_turn_rate;      // Bezwładność: max. stopnie skrętu na sekundę
};

Target targets[3] = {
  // ID, Lat, Lon, COG, SOG, Nazwa, Promień, BaseSOG, MaxSOG, ReturnFlag, TargetCOG, PlannedCOG, AvoidingFlag, LastDecision, TurnRate
  {1, 55.5500, 18.0500, 210.0, 16.0, "PROM",      15.0, 16.0, 24.0, false, 210.0, 210.0, false, 0, 1.5}, 
  {2, 55.4500, 17.9500,  45.0, 10.0, "TANKOWIEC", 20.0, 10.0, 15.0, false,  45.0,  45.0, false, 0, 0.2}, 
  {3, 55.5000, 18.1000,   0.0,  0.0, "RYBAK",      6.0,  0.0, 12.0, false,   0.0,   0.0, false, 0, 4.0}  
};

// --- STAN SYMULATORA ---
bool enableUDP = true, enableTCP = true, debug_mode = false;
bool navMode = false, engine_on = false; 
float target_nav_cog = -1.0, target_rpm = 900.0, current_rpm = 0.0;
float target_wind_dir = 0.0;    
float target_wind_speed = 0.0; 
float true_wind_dir = 0.0, true_wind_speed = 0.0;
float current_lat = 55.5000, current_lon = 18.0000; 
float current_cog = 0.0, current_sog = 0.0, current_depth = 30.0;
int sendInterval = 1;
float total_log_nm = 0.0;   // całkowita droga od uruchomienia
float trip_log_nm   = 0.0;  // log trip

// --- ZMIENNE LOKALNE FIZYKI ---
//float target_wind_dir_local = 0.0, target_wind_speed_local = 0.0;
float target_sog = 0.0, target_depth = 30.0;
unsigned long lastTick = 0;

// =========================================================================
// POMOCNICZA FUNKCJA PARSOWANIA JSON
// =========================================================================
String getJsonValue(const String& json, const String& key) {
  String search = "\"" + key + "\":";
  int start = json.indexOf(search);
  if (start == -1) return "";
  start += search.length();
  if (json[start] == '"') {
    start++;
    int end = json.indexOf('"', start);
    return json.substring(start, end);
  } else {
    int end = json.indexOf(',', start);
    if (end == -1) end = json.indexOf('}', start);
    return json.substring(start, end);
  }
}

// =========================================================================
// REALISTYCZNA FIZYKA JACHTU (Prędkość kadłuba ~9.5 kn)
// =========================================================================
float calculateTargetSOG(float cog, float twd, float tws, bool eng_on, float rpm) {
  float twa = twd - cog;
  while (twa <= -180.0) twa += 360.0; 
  while (twa > 180.0) twa -= 360.0;
  
  float abs_twa = abs(twa);
  
  // 1. Sprawność żagli w zależności od kąta wiatru (Biegunowa krzywa prędkości)
  float eff = 0.0;
  if (abs_twa < 35.0) eff = 0.0; // Kąt martwy (łopoczące żagle)
  else if (abs_twa < 90.0) eff = ((abs_twa - 35.0) / 55.0) * 0.50; // Bajdewind -> Półwiatr
  else if (abs_twa <= 135.0) eff = 0.50 + (((abs_twa - 90.0) / 45.0) * 0.25); // Baksztag (najwydajniejszy)
  else eff = 0.75 - (((abs_twa - 135.0) / 45.0) * 0.15); // Fordewind (nieco wolniej przez zasłanianie żagli)
  
  // 2. Prędkość z żagli z uwzględnieniem oporu kadłuba
  float max_hull_speed = 9.5; 
  float sog_sail = tws * eff;
  
  // Refowanie żagli i ściana wody: Powyżej prędkości kadłuba wiatr nie daje już prędkości, tylko przechył
  if (sog_sail > max_hull_speed) {
    // "Miękkie odcięcie" - pozwala na chwilowy, minimalny zjazd z fali (np. max do 10.5 kn przy sztormie), ale brutalnie hamuje wzrost
    sog_sail = max_hull_speed + (sog_sail - max_hull_speed) * 0.1; 
  }
  
  // 3. Prędkość z silnika (Skalowana od biegu jałowego do max obrotów)
  float sog_engine = 0.0;
  if (eng_on && rpm >= 900.0) {
    // 900 RPM = 1.5 kn | 4000 RPM = 8.5 kn
    sog_engine = 1.5 + ((rpm - 900.0) / 3100.0) * 7.0; 
  }
  
  // 4. Hybryda (Motorsailing)
  float final_sog = max(sog_sail, sog_engine); // Zasadniczo szybszy napęd "wygrywa"
  
  // Jeśli płyniemy na żaglach i pomagamy sobie silnikiem, dodajemy mały bonus, 
  // ale nigdy nie przekraczamy fizycznej prędkości kadłuba.
  if (sog_sail > 2.0 && eng_on && rpm > 1000.0) {
    final_sog += 1.0; 
  }
  
  // Twardy limit bezpieczeństwa na wypadek błędów matematycznych
  return constrain(final_sog, 0.0f, 11.0f);
}

void calculateApparentWind(float cog, float sog, float twd, float tws, float &awa, float &aws) {
  float twa_rad = (twd - cog) * PI / 180.0;
  float w_x = tws * sin(twa_rad);
  float w_y = tws * cos(twa_rad) + sog; 
  aws = sqrt(w_x*w_x + w_y*w_y);
  awa = atan2(w_x, w_y) * 180.0 / PI;
  if (awa < 0) awa += 360.0;
}

void addNMEAChecksumToLine(const char* baseLine, char* outputLine) {
  int checksum = 0;
  for (int i = 1; baseLine[i] != 0; i++) checksum ^= baseLine[i];
  sprintf(outputLine, "%s*%02X\r\n", baseLine, checksum);
}


// =========================================================================
// MATEMATYKA ARPA (Przewidywanie Kolizji: CPA / TCPA)
// Oblicza wektory i zwraca TRUE, jeśli przecinamy się w odległości < 2 NM, 
// a czas zderzenia (TCPA) to od 0 do 15 minut.
// =========================================================================
bool checkCollisionRisk_ARPA(float lat1, float lon1, float sog1, float cog1, 
                             float lat2, float lon2, float sog2, float cog2) {
  
  // 1. Konwersja współrzędnych na lokalny układ kartezjański (XY) w milach morskich.
  // Jacht jest punktem odniesienia (0,0).
  float dx_nm = (lon2 - lon1) * cos(lat1 * PI / 180.0) * 60.0;
  float dy_nm = (lat2 - lat1) * 60.0;

  // 2. Rozkład wektorów prędkości obu jednostek na składowe X i Y (Węzły = NM/h).
  float v1_x = sog1 * sin(cog1 * PI / 180.0);
  float v1_y = sog1 * cos(cog1 * PI / 180.0);
  float v2_x = sog2 * sin(cog2 * PI / 180.0);
  float v2_y = sog2 * cos(cog2 * PI / 180.0);

  // 3. Obliczenie wektora prędkości względnej (Relative Velocity).
  float dv_x = v2_x - v1_x;
  float dv_y = v2_y - v1_y;
  float v_rel2 = dv_x*dv_x + dv_y*dv_y;

  if (v_rel2 < 0.01) return false; // Statki stoją w miejscu względem siebie

  // 4. TCPA (Time to Closest Point of Approach) - czas do najbliższego zbliżenia.
  // Ujemny wynik oznacza, że statki już się mijają.
  float tcpa_hours = -(dx_nm * dv_x + dy_nm * dv_y) / v_rel2;
  float tcpa_min = tcpa_hours * 60.0;

  if (tcpa_min < 0.0 || tcpa_min > 15.0) return false;

  // 5. CPA (Closest Point of Approach) - najmniejsza przewidywana odległość między statkami.
  float cpa_x = dx_nm + dv_x * tcpa_hours;
  float cpa_y = dy_nm + dv_y * tcpa_hours;
  float cpa_nm = sqrt(cpa_x*cpa_x + cpa_y*cpa_y);

  if (cpa_nm < 2.0) {
    return true;
  }

  return false;
}

// =========================================================================
// EKRAN TFT i GUI
// =========================================================================
void initDisplayStatic() { 
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1); tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(2, 0); 
  
  if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
    tft.print(" IP: "); tft.println(WiFi.softAPIP());
  } else {
    tft.print(" IP: "); tft.println(WiFi.localIP());
  }
  tft.drawFastHLine(0, 12, 135, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setCursor(2, 62); tft.print("KNOTS SOG");
  tft.setTextColor(TFT_ORANGE, TFT_BLACK); tft.setCursor(2, 117); tft.print("DEGREE COG");
}

void updateDisplay() {
  float l_sog, l_cog, l_depth, l_wind;
  bool l_nav, l_udp, l_tcp, l_dbg;
  
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    l_sog = current_sog; l_cog = current_cog; l_depth = current_depth; l_wind = true_wind_speed;
    l_nav = navMode; l_udp = enableUDP; l_tcp = enableTCP; l_dbg = debug_mode;
    xSemaphoreGive(dataMutex);
  }

  sprSOG.fillSprite(TFT_BLACK);
  sprSOG.setTextSize(4); sprSOG.setTextColor(TFT_WHITE, TFT_BLACK); 
  sprSOG.setCursor(0, 0); sprSOG.printf("%.1f", l_sog); sprSOG.pushSprite(2, 30);

  sprCOG.fillSprite(TFT_BLACK);
  sprCOG.setTextSize(4); sprCOG.setTextColor(TFT_ORANGE, TFT_BLACK); 
  sprCOG.setCursor(0, 0); sprCOG.printf("%03.0f", l_cog); sprCOG.pushSprite(2, 85);

  sprInfo.fillSprite(TFT_BLACK);
  sprInfo.setTextSize(2); sprInfo.setTextColor(TFT_CYAN, TFT_BLACK); 
  sprInfo.setCursor(0, 0); sprInfo.printf("DEP:%.1f m", l_depth);
  sprInfo.setCursor(0, 16); sprInfo.printf("TWS:%.1f kn", l_wind); sprInfo.pushSprite(2, 135);

  sprStat.fillSprite(TFT_BLACK);
  sprStat.setTextSize(1); sprStat.setTextColor(TFT_GREEN, TFT_BLACK); 
  sprStat.setCursor(0, 0); sprStat.printf("U:%d T:%d D:%d", l_udp, l_tcp, l_dbg);
  sprStat.setCursor(80, 0);
  sprStat.setTextColor(l_nav ? TFT_MAGENTA : TFT_DARKGREY, TFT_BLACK);
  sprStat.print(l_nav ? "NAVI" : "MAN."); 
  sprStat.pushSprite(2, 175);
}

void guiTaskCode(void * pvParameters) {
  initDisplayStatic(); updateDisplay();
  bool lastL = HIGH, lastR = HIGH;
  unsigned long lastDispUpdate = 0, debounceL = 0, debounceR = 0; 
  unsigned long bothPressedTime = 0; 
  int lastCountdownSec = -1; // Zmienna zapobiegająca migotaniu ekranu

  for(;;) {
    bool currentL = digitalRead(BTN_LEFT);
    bool currentR = digitalRead(BTN_RIGHT);

    // === SPRAWDZANIE RESETU FABRYCZNEGO (Oba przyciski wciśnięte) ===
    if (currentL == LOW && currentR == LOW) {
      if (bothPressedTime == 0) {
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
      } else {
        unsigned long elapsed = millis() - bothPressedTime;
        if (elapsed > 5000) {
          // Minęło 5 sekund - TWARDY RESET
          Serial.println("\n[RESET] Wykryto twardy reset! Przywracanie ustawien fabrycznych...");
          
          tft.fillScreen(TFT_RED);
          tft.setTextSize(2);
          tft.setCursor(10, 50);
          tft.println("FACTORY");
          tft.setCursor(10, 80);
          tft.println("RESET...");
          
          preferences.clear(); 
          delay(2000); 
          ESP.restart(); 
        } else {
          // Trwa odliczanie
          int secLeft = 5 - (elapsed / 1000);
          // Odśwież cyfrę tylko, jeśli się zmieniła
          if (secLeft != lastCountdownSec && secLeft > 0) {
            lastCountdownSec = secLeft;
            tft.fillRect(0, 70, 135, 60, TFT_RED); // Czyścimy tylko obszar starej cyfry
            tft.setCursor(55, 80);
            tft.println(lastCountdownSec);
          }
        }
      }
    } else {
      // Jeśli użytkownik puści przyciski przed czasem (przerwie reset)
      if (bothPressedTime != 0) {
        bothPressedTime = 0; 
        lastCountdownSec = -1;
        
        // Szybkie przywrócenie normalnego interfejsu
        initDisplayStatic(); 
        updateDisplay();
      }
    }

    // === STANDARDOWA OBSŁUGA ZMIANY KURSU ===
    // (Wykona się tylko, jeśli nie trwa proces resetowania)
    if (bothPressedTime == 0) {
      if (currentL == LOW && lastL == HIGH && (millis() - debounceL > 150)) { 
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          navMode = false; current_cog -= 10.0;
          if (current_cog < 0.0) current_cog += 360.0;
          xSemaphoreGive(dataMutex);
        }
        updateDisplay(); debounceL = millis(); 
      }

      if (currentR == LOW && lastR == HIGH && (millis() - debounceR > 150)) { 
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          navMode = false; current_cog += 10.0;
          if (current_cog >= 360.0) current_cog -= 360.0;
          xSemaphoreGive(dataMutex);
        }
        updateDisplay(); debounceR = millis(); 
      }
    }

    lastL = currentL;
    lastR = currentR;

    // Odświeżanie standardowych danych co 500ms (tylko gdy nie ma resetu)
    if (bothPressedTime == 0 && millis() - lastDispUpdate > 500) { 
      updateDisplay(); 
      lastDispUpdate = millis(); 
    }
    
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}


// =========================================================================
// OBSŁUGA ZDARZEŃ WEBSOCKET (KOMENDY Z PRZEGLĄDARKI)
// =========================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  
  // --- NOWE: Natychmiastowe wysłanie danych przy podłączeniu przeglądarki ---
  if (type == WS_EVT_CONNECT) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      char wsJson[150];
      // DODANO "s" (sog) i "ts" (target_sog) z jedną cyfrą po przecinku (%.1f)
      snprintf(wsJson, sizeof(wsJson), 
          "{\"c\":%.1f,\"s\":%.1f,\"ts\":%.1f,\"n\":%d,\"e\":%d,\"r\":%d}",
          current_cog, current_sog, target_sog, navMode ? 1 : 0, engine_on ? 1 : 0, (int)current_rpm);
      client->text(wsJson); 
      xSemaphoreGive(dataMutex);
    }
  }
  
  // --- STARE: Odbieranie komend z przycisków ---
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      String msg = String((char*)data, len);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        if (msg == "NAV") {
          navMode = !navMode;
        } else if (msg == "L1") {
          navMode = false; current_cog -= 1.0;
        } else if (msg == "R1") {
          navMode = false; current_cog += 1.0;
        } else if (msg == "L10") {
          navMode = false; current_cog -= 10.0;
        } else if (msg == "R10") {
          navMode = false; current_cog += 10.0;
        } else if (msg.startsWith("ENG:")) {
          int idx1 = msg.indexOf(':');
          int idx2 = msg.lastIndexOf(':');
          if (idx1 != -1 && idx2 != -1 && idx1 != idx2) {
            engine_on = msg.substring(idx1+1, idx2).toInt() == 1;
            target_rpm = msg.substring(idx2+1).toFloat();
            current_rpm = engine_on ? target_rpm : 0.0; 
          }
        }
        if (current_cog < 0.0) current_cog += 360.0;
        if (current_cog >= 360.0) current_cog -= 360.0;
        xSemaphoreGive(dataMutex);
      }
    }
  }
}

// =========================================================================
// WEB SERVER – ASYNCHRONICZNA WERSJA (stabilna 24/7+)
// =========================================================================
void setupWebServer() {
  // === WEBSOCKET ===
  ws.onEvent(onWsEvent);
  webServer.addHandler(&ws);

  // === STRONY HTML (Ładowane z pamięci LittleFS) ===
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  webServer.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/config.html", "text/html");
  });

  webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204);
  });

  // === SKANOWANIE WIFI ===
  webServer.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
    int n = WiFi.scanNetworks();
    String json = "[";
    for(int i = 0; i < n; i++) {
      if(i > 0) json += ",";
      json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]";
    request->send(200, "application/json", json);
  });

// === ZAPIS KONFIGURACJI WIFI (POST JSON) ===
  webServer.on("/saveconfig", HTTP_POST, 
    [](AsyncWebServerRequest *request){}, 
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
      if (index + len == total) {
        String body = String((char*)data, len);

        String ssid_new = getJsonValue(body, "ssid");
        String pass_new = getJsonValue(body, "pass");
        bool   ap_new   = getJsonValue(body, "ap") == "true";

        bool wifiChanged = false;

        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
          if (ap_new != useAP_mode || 
              (ssid_new.length() > 0 && ssid_new != wifi_ssid) ||
              (ssid_new.length() > 0 && pass_new != wifi_pass)) {
            wifiChanged = true;
          }

          preferences.putString("wifi_ssid", ssid_new);
          preferences.putString("wifi_pass", pass_new);
          preferences.putBool("useAP", ap_new);

          useAP_mode = ap_new;
          if (ssid_new.length() > 0) wifi_ssid  = ssid_new;
          if (pass_new.length() > 0) wifi_pass  = pass_new;

          xSemaphoreGive(dataMutex);
        }

        if (wifiChanged) {
          Serial.println("[CONFIG] Zmiana ustawień WiFi → restart");
          request->send(200, "text/plain", "Zapisano WiFi! ESP32 się restartuje...");
          delay(2000);
          ESP.restart();
        } else {
          request->send(200, "text/plain", "Zapisano konfiguracje (bez zmian sieciowych)");
        }
      }
    }
  );

  // === BŁYSKAWICZNY ZAPIS USTAWIEŃ (TŁO) ===
  webServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      if (request->hasParam("u")) {
        enableUDP = request->getParam("u")->value() == "1";
        preferences.putBool("udp", enableUDP);
      }
      if (request->hasParam("t")) {
        enableTCP = request->getParam("t")->value() == "1";
        preferences.putBool("tcp", enableTCP);
      }
      if (request->hasParam("d")) {
        debug_mode = request->getParam("d")->value() == "1";
        preferences.putBool("dbg", debug_mode);
      }
      if (request->hasParam("i")) {
        sendInterval = request->getParam("i")->value().toInt();
        if (sendInterval < 1) sendInterval = 1;
        preferences.putInt("interval", sendInterval);
      }
      xSemaphoreGive(dataMutex);
    }
    request->send(200);
  });

  // === ENDPOINT /data ===
  webServer.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){ 
    float l_cog = 0.0;
    int   l_current_rpm = 0;
    int   l_target_rpm = 900;
    int   l_int = 1;
    bool  l_nav = false;
    bool  l_udp = true;
    bool  l_tcp = true;
    bool  l_dbg = false;
    bool  l_eng = false; // <--- DODANA ZMIENNA

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
        l_cog          = current_cog;
        l_current_rpm  = (int)current_rpm;
        l_target_rpm   = (int)target_rpm;
        l_nav          = navMode;
        l_udp          = enableUDP;
        l_tcp          = enableTCP;
        l_dbg          = debug_mode;
        l_int          = sendInterval;
        l_eng          = engine_on; // <--- ODCZYT STANU SILNIKA
        xSemaphoreGive(dataMutex); 
    }

    char json[220];
    snprintf(json, sizeof(json), 
        "{\"cog\":%.1f,\"n\":%d,\"e\":%d,\"rpm\":%d,\"target_rpm\":%d,\"u\":%s,\"t\":%s,\"d\":%s,\"i\":%d}",
        l_cog, 
        l_nav ? 1 : 0, 
        l_eng ? 1 : 0, // <--- WYSYŁANIE STANU (jako "e")
        l_current_rpm,      
        l_target_rpm,       
        l_udp ? "true" : "false",
        l_tcp ? "true" : "false",
        l_dbg ? "true" : "false",
        l_int);

    request->send(200, "application/json", json); 
  });

  // === ZMIANA KURSU ===
  webServer.on("/l10", HTTP_GET, [](AsyncWebServerRequest *request){
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
      navMode = false; current_cog -= 10.0;
      if (current_cog < 0.0) current_cog += 360.0;
      xSemaphoreGive(dataMutex); 
    }
    request->send(200);
  });
  webServer.on("/l1", HTTP_GET, [](AsyncWebServerRequest *request){
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
      navMode = false; current_cog -= 1.0;
      if (current_cog < 0.0) current_cog += 360.0;
      xSemaphoreGive(dataMutex); 
    }
    request->send(200);
  });
  webServer.on("/r1", HTTP_GET, [](AsyncWebServerRequest *request){
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
      navMode = false; current_cog += 1.0;
      if (current_cog >= 360.0) current_cog -= 360.0;
      xSemaphoreGive(dataMutex); 
    }
    request->send(200);
  });
  webServer.on("/r10", HTTP_GET, [](AsyncWebServerRequest *request){
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
      navMode = false; current_cog += 10.0;
      if (current_cog >= 360.0) current_cog -= 360.0;
      xSemaphoreGive(dataMutex); 
    }
    request->send(200);
  });

  // === NAV / AUTOPILOT ===
  webServer.on("/nav", HTTP_GET, [](AsyncWebServerRequest *request){ 
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
      navMode = !navMode; 
      xSemaphoreGive(dataMutex); 
    } 
    request->send(200); 
  });

  // === SILNIK ===
  webServer.on("/engine", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("s") && request->hasParam("r")) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
        engine_on = request->getParam("s")->value() == "1"; 
        target_rpm = request->getParam("r")->value().toFloat(); 
        
        // BŁYSKAWICZNA AKTUALIZACJA: zapobiega "migotaniu" starych danych w przeglądarce
        if (engine_on) {
          current_rpm = target_rpm; 
        } else {
          current_rpm = 0.0;
        }
        
        xSemaphoreGive(dataMutex); 
      }
    }
    request->send(200);
  });

  webServer.begin();   // <<< WAŻNE – start asynchronicznego serwera
  Serial.println("[WEB] AsyncWebServer uruchomiony na porcie 80");
}

// =========================================================================
// GENERATORY AIS (bez zmian)
// =========================================================================
class FastAISBitWriter {
private:
    uint8_t payload[40]; 
    int bitIndex;
public:
    FastAISBitWriter() { memset(payload, 0, sizeof(payload)); bitIndex = 0; }
    void pushBits(uint32_t value, int numBits) {
        for (int i = numBits - 1; i >= 0; i--) {
            if ((value >> i) & 1) payload[bitIndex / 8] |= (1 << (7 - (bitIndex % 8)));
            bitIndex++;
        }
    }
    void pushString(const char* str, int maxChars) {
        for (int i = 0; i < maxChars; i++) {
            char c = (i < strlen(str)) ? str[i] : '@'; 
            uint32_t val = 0;
            if (c >= '@' && c <= '_') val = c - 0x40; else if (c >= ' ' && c <= '?') val = c - 0x20 + 32;
            pushBits(val, 6);
        }
    }
    void encodeToAscii(char* outputAscii, int& padding) {
        int numChars = (bitIndex + 5) / 6;
        padding = (numChars * 6) - bitIndex;
        for (int i = 0; i < numChars; i++) {
            uint8_t sixBits = 0;
            for (int b = 0; b < 6; b++) {
                int absBit = (i * 6) + b;
                if (absBit < bitIndex && (payload[absBit / 8] & (1 << (7 - (absBit % 8))))) { sixBits |= (1 << (5 - b)); }
            }
            sixBits += 0x30; if (sixBits > 0x57) sixBits += 0x08;
            outputAscii[i] = sixBits;
        }
        outputAscii[numChars] = '\0';
    }
};

// ZMIANA: Dodaliśmy parametr "float heading"
void generateAIVDM(int mmsi, float lat, float lon, float sog, float cog, float heading, char* outputLine) {
    FastAISBitWriter aisWriter;
    int32_t i_lon = (int32_t)(lon * 600000.0); 
    int32_t i_lat = (int32_t)(lat * 600000.0);
    uint32_t i_sog = (uint32_t)(sog * 10.0); if (i_sog > 1022) i_sog = 1022; 
    uint32_t i_cog = (uint32_t)(cog * 10.0); if (i_cog > 3599) i_cog = 3599;

    // NOWE: Heading musi być liczbą całkowitą 0-359. 511 oznacza "brak danych".
    uint32_t i_hdg = (uint32_t)heading; 
    if (heading > 359.0 || heading < 0.0) i_hdg = 511; 

    aisWriter.pushBits(1, 6); 
    aisWriter.pushBits(0, 2); 
    aisWriter.pushBits(mmsi, 30);       
    aisWriter.pushBits(0, 4); 
    aisWriter.pushBits(0, 8); 
    aisWriter.pushBits(i_sog, 10);      
    aisWriter.pushBits(0, 1); 
    aisWriter.pushBits(i_lon, 28); 
    aisWriter.pushBits(i_lat, 27);      
    aisWriter.pushBits(i_cog, 12); 
    aisWriter.pushBits(i_hdg, 9);    // TU WPISUJEMY HEADING (9 bitów)
    aisWriter.pushBits(60, 6); 
    aisWriter.pushBits(0, 2); 
    aisWriter.pushBits(0, 3); 
    aisWriter.pushBits(0, 1); 
    aisWriter.pushBits(0, 19);          

    char aisPayload[45]; int padding; aisWriter.encodeToAscii(aisPayload, padding);
    char baseLine[120]; sprintf(baseLine, "!AIVDM,1,1,,A,%s,%d", aisPayload, padding);
    
    // Używamy outputLine (to był ten błąd kompilacji wcześniej)
    addNMEAChecksumToLine(baseLine, outputLine);
}
// =========================================================================
// GENERATOR AIS TYPE 24A (Static Data Part A)
// =========================================================================
void generateType24A(int mmsi, const char* name, char* outputLine) {
    FastAISBitWriter aisWriter;
    aisWriter.pushBits(24, 6); 
    aisWriter.pushBits(0, 2); 
    aisWriter.pushBits(mmsi, 30);     
    aisWriter.pushBits(0, 2); 
    aisWriter.pushString(name, 20);   

    char aisPayload[40]; int padding; aisWriter.encodeToAscii(aisPayload, padding);
    char baseLine[100]; sprintf(baseLine, "!AIVDM,1,1,,A,%s,%d", aisPayload, padding);
    addNMEAChecksumToLine(baseLine, outputLine);
}

// =========================================================================
// GENERATOR AIS TYPE 24B (Static Data Part B)
// =========================================================================
void generateType24B(int mmsi, const char* callsign, uint8_t shipType,
                     float length_m, float beam_m, char* outputLine) {
    
    FastAISBitWriter aisWriter;
    
    // Message 24 Part B
    aisWriter.pushBits(24, 6);          // Message ID
    aisWriter.pushBits(0, 2);           // Repeat Indicator
    aisWriter.pushBits(mmsi, 30);       // MMSI
    aisWriter.pushBits(1, 2);           // Part Number = 1 (B)
    aisWriter.pushBits(shipType, 8);    // Ship Type
    aisWriter.pushBits(0, 42);  // Vendor ID (puste)
    aisWriter.pushString(callsign, 7); // Call Sign (7 znaków, 6-bit)
    
    // Wymiary (w metrach)
    uint32_t dimA = (uint32_t)(length_m * 0.1f);   // bow
    uint32_t dimB = 0;                              // stern (dla prostoty 0)
    uint32_t dimC = (uint32_t)(beam_m * 0.1f / 2); // port
    uint32_t dimD = dimC;                           // starboard
    
    aisWriter.pushBits(dimA, 9);
    aisWriter.pushBits(dimB, 9);
    aisWriter.pushBits(dimC, 6);
    aisWriter.pushBits(dimD, 6);
    aisWriter.pushBits(0, 6);   // Spare
    
    char aisPayload[45];
    int padding;
    aisWriter.encodeToAscii(aisPayload, padding);
    
    char baseLine[120];
    sprintf(baseLine, "!AIVDM,1,1,,A,%s,%d", aisPayload, padding);
    addNMEAChecksumToLine(baseLine, outputLine);
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(921600); 
  randomSeed(esp_random()); 

  // <<< CZAS AWARYJNY USTAWIANY ZAWSZE NA STARCIE >>>
  struct timeval tv;
  tv.tv_sec = 1767268800; // 2026-01-01 12:00:00 UTC
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  Serial.println("[SYSTEM] Ustawiono startowy czas awaryjny: 2026-01-01 12:00:00 UTC");

  // --- INICJALIZACJA SYSTEMU PLIKÓW (LittleFS) ---
  if (!LittleFS.begin(true)) {
  Serial.println("[BŁĄD] Wystapil problem podczas montowania systemu LittleFS");
  // Mimo błędu symulator może działać, ale strony WWW się nie załadują.
  } else {
  Serial.println("[SYSTEM] LittleFS zamontowany poprawnie.");
  }
  
  // --- INICJALIZACJA MUTEXA DANYCH ---
  dataMutex = xSemaphoreCreateMutex();
  
  // --- INICJALIZACJA TFT ---
  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  tft.init(); tft.setRotation(0); 
  pinMode(BTN_LEFT, INPUT_PULLUP); pinMode(BTN_RIGHT, INPUT); 

  sprSOG.createSprite(100, 30); sprCOG.createSprite(100, 30);
  sprInfo.createSprite(130, 32); sprStat.createSprite(130, 12);

  //preferences.clear();   // odkomentuj tylko raz, żeby wyczyścić stare dane

  preferences.begin("jacht-config", false);

  // Wczytanie konfiguracji WiFi i ustawień
  wifi_ssid    = preferences.getString("wifi_ssid", "");
  wifi_pass    = preferences.getString("wifi_pass", "");
  useAP_mode   = preferences.getBool("useAP", false);
  enableUDP    = preferences.getBool("udp", true);
  enableTCP    = preferences.getBool("tcp", true);
  debug_mode   = preferences.getBool("dbg", false);
  sendInterval = preferences.getInt("interval", 1); 

  Serial.println("\n=== KONFIGURACJA WCZYTANA ===");
  Serial.printf("SSID      : '%s'\n", wifi_ssid.c_str());
  Serial.printf("AP mode   : %s\n", useAP_mode ? "TAK" : "NIE");
  Serial.printf("Debug     : %s\n", debug_mode ? "TAK" : "NIE");
  Serial.printf("Interval  : %d s\n", sendInterval);
  Serial.printf("SSID: '%s' | AP: %s | Debug: %s | Interval: %d s\n", 
                wifi_ssid.c_str(), useAP_mode?"TAK":"NIE", debug_mode?"TAK":"NIE", sendInterval);

  target_wind_speed = random(40, 270) / 10.0; 
  target_wind_dir = random(0, 360);
  true_wind_speed = target_wind_speed; 
  true_wind_dir = target_wind_dir;

  // --- LOGIKA POŁĄCZENIA WIFI ---
  if (useAP_mode || wifi_ssid.length() == 0) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid_default, ap_password);
    Serial.println("\n[SIEC] Tryb AP uruchomiony z wyboru lub braku SSID.");
    Serial.print("[SIEC] IP: "); Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    bool connected = false;

    // Pętla 3 prób połączenia
    for (int attempt = 1; attempt <= 3; attempt++) {
      Serial.printf("\n[SIEC] Łączenie z siecią: %s (Próba %d/3)\n", wifi_ssid.c_str(), attempt);
      
      WiFi.disconnect(); // Rozłącz przed nową próbą, resetuje stan modułu
      delay(100);
      WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
      
      unsigned long startAttempt = millis();
      // Czekaj maksymalnie 10 sekund na jedną próbę
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
        delay(500); 
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        break; // Połączono pomyślnie - przerywamy pętlę
      } else {
        Serial.println(" Nieudana.");
      }
    }

    // Jeśli po 3 próbach nadal nie ma połączenia - uruchom AP (Fallback)
    if (!connected) {
      Serial.println("\n[SIEC] Błąd połączenia po 3 próbach - uruchamiam tryb ratunkowy AP");
      WiFi.mode(WIFI_AP); // Wymuszenie trybu AP
      WiFi.softAP(ap_ssid_default, ap_password);
      Serial.print("[SIEC] IP AP: "); Serial.println(WiFi.softAPIP());
    } else {
      Serial.print("\n[SIEC] Połączono! IP: "); Serial.println(WiFi.localIP());
      // Włącz automatyczne wznawianie połączenia na wypadek późniejszej utraty zasięgu
      WiFi.setAutoReconnect(true); 
      // Synchronizacja czasu NTP (UTC)
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      Serial.println("[SYSTEM] Uruchomiono synchronizację czasu NTP (UTC).");

    }
  }

  udp.begin(port); 
  nmeaServer.begin(); 
  setupWebServer();

  // --- Type 24A (pełne dane statyczne) ---
  generateType24A(261000001, targets[0].name, staticAIS_PromA);
  generateType24A(261000002, targets[1].name, staticAIS_TankowiecA);
  generateType24A(261000003, targets[2].name, staticAIS_RybakA);
  // --- Type 24B (pełne dane statyczne) ---
  generateType24B(261000001, "PROM1",  60, 150.0, 25.0, staticAIS_PromB);      // Passenger
  generateType24B(261000002, "TANK2",  80, 220.0, 32.0, staticAIS_TankowiecB); // Tanker
  generateType24B(261000003, "FISH3",  30,  22.0,  7.0, staticAIS_RybakB);     // Fishing

  esp_task_wdt_init(10, true);        // 10 sekund timeout
  esp_task_wdt_add(NULL);             // dodaje bieżący task (loop)

  xTaskCreatePinnedToCore(guiTaskCode, "GUITask", 12288, NULL, 1, &GUITask, 0); 
}


// =========================================================================
// GŁÓWNA PĘTLA – WERSJA W PEŁNI ZOPTYMALIZOWANA
// =========================================================================
void loop() {
  // === 1. Obsługa komend autopilota z OpenCPN (RMB) - Asynchronicznie ===
  static String tcpBuffer = ""; 
  for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
    if (tcpClients[i] && tcpClients[i].connected() && tcpClients[i].available()) {
      while (tcpClients[i].available()) {
        char c = tcpClients[i].read();
        tcpBuffer += c;
        
        if (c == '\n') {
          if (tcpBuffer.indexOf("RMB") > 0) {
            int commaIndex[15]; int count = 0;
            for (int k = 0; k < tcpBuffer.length(); k++) { 
              if (tcpBuffer[k] == ',') { 
                commaIndex[count] = k; count++; 
                if (count >= 15) break; 
              } 
            }
            if (count >= 12 && tcpBuffer.substring(commaIndex[0]+1, commaIndex[1]) == "A") { 
              String bearingStr = tcpBuffer.substring(commaIndex[10]+1, commaIndex[11]);
              if (bearingStr.length() > 0) {
                if (xSemaphoreTake(dataMutex, portMAX_DELAY)) { 
                  target_nav_cog = bearingStr.toFloat(); 
                  xSemaphoreGive(dataMutex); 
                }
                Serial.printf(">>> [AUTOPILOT] OpenCPN narzuca korekte kursu: %.1f st.\n", bearingStr.toFloat());
              }
            }
          }
          tcpBuffer = ""; // Wyczyść bufor po przetworzeniu linii
        }
      }
    }
  }

  // === 2 i 3. Zarządzanie wieloma klientami TCP ===
  bool l_tcp_global = enableTCP; // Odczyt tylko w celu włączenia/wyłączenia serwera
  if (l_tcp_global) {
    // Akceptacja nowych
    if (nmeaServer.hasClient()) {
      bool clientAdded = false;
      for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (!tcpClients[i] || !tcpClients[i].connected()) {
          if (tcpClients[i]) tcpClients[i].stop();
          tcpClients[i] = nmeaServer.available();
          Serial.printf("[TCP] Nowy klient podłączony do slotu %d z IP: %s\n", 
                        i, tcpClients[i].remoteIP().toString().c_str());
          clientAdded = true;
          break;
        }
      }
      if (!clientAdded) {
        WiFiClient rejectClient = nmeaServer.available();
        rejectClient.stop();
        Serial.println("[TCP] Odrzucono klienta - brak wolnych slotów.");
      }
    }
    
    // Czyszczenie martwych
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
      if (tcpClients[i] && !tcpClients[i].connected()) {
        tcpClients[i].stop();
      }
    }
  }

  // === 4. Fizyka + wysyłanie NMEA – tylko co sendInterval sekund ===
  if (millis() - lastTick >= (unsigned long)(sendInterval * 1000)) {
    lastTick = millis();
    
    // --- Czas i data UTC ---
    time_t nowTime;
    struct tm timeinfo;
    time(&nowTime);
    gmtime_r(&nowTime, &timeinfo); 

    char timeBuf[16]; 
    strftime(timeBuf, sizeof(timeBuf), "%H%M%S.00", &timeinfo);
    char dateBuf[8];
    strftime(dateBuf, sizeof(dateBuf), "%d%m%y", &timeinfo);

    // =================================================================
    // --- FIZYKA (ZOPTYMALIZOWANA DLA FREERTOS) ---
    // =================================================================

    // KROK 1: Deklaracja zmiennych lokalnych
    float l_tws, l_twd, l_tgt_ws, l_tgt_wd;
    float l_depth, l_tgt_depth;
    float l_rpm, l_tgt_rpm;
    float l_cog, l_tgt_nav_cog;
    float l_sog, l_tgt_sog;
    float l_lat, l_lon;
    float l_total_log, l_trip_log;
    bool l_eng, l_nav, l_udp, l_tcp, l_dbg;
    int l_interval;

    // KROK 2: Odczyt z Mutexa
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      l_tws = true_wind_speed;         l_twd = true_wind_dir;
      l_tgt_ws = target_wind_speed;    l_tgt_wd = target_wind_dir;
      l_depth = current_depth;         l_tgt_depth = target_depth;
      l_rpm = current_rpm;             l_tgt_rpm = target_rpm;
      l_cog = current_cog;             l_tgt_nav_cog = target_nav_cog;
      l_sog = current_sog;             l_tgt_sog = target_sog;
      l_lat = current_lat;             l_lon = current_lon;
      l_total_log = total_log_nm;      l_trip_log = trip_log_nm;
      l_eng = engine_on;               l_nav = navMode;
      l_interval = sendInterval;
      l_udp = enableUDP;               l_tcp = enableTCP;
      l_dbg = debug_mode;
      xSemaphoreGive(dataMutex); 
    }

    // KROK 3: Obliczenia w tle (bez blokowania)
    if (random(0, 15) == 0) { 
      l_tgt_ws += random(-30, 31) / 10.0; 
      l_tgt_ws = constrain(l_tgt_ws, 4.0, 35.0);
      l_tgt_wd += random(-15, 16); 
      if (l_tgt_wd >= 360.0) l_tgt_wd -= 360.0; 
      if (l_tgt_wd < 0.0) l_tgt_wd += 360.0;
    }
    
    l_tws += (l_tgt_ws - l_tws) * 0.1;
    float dir_diff = l_tgt_wd - l_twd;
    if (dir_diff > 180.0) dir_diff -= 360.0; 
    if (dir_diff < -180.0) dir_diff += 360.0;
    if (abs(dir_diff) > 0.5) { 
      l_twd += dir_diff * 0.05; 
      if (l_twd >= 360.0) l_twd -= 360.0; 
      if (l_twd < 0.0) l_twd += 360.0; 
    }

    if (random(0, 10) == 0) l_tgt_depth = random(20, 800) / 10.0;
    l_depth += (l_tgt_depth - l_depth) * 0.05;      

    l_rpm = l_eng ? l_tgt_rpm + random(-15, 16) : 0.0;

    if (l_nav && l_tgt_nav_cog >= 0.0) {
      float diff = l_tgt_nav_cog - l_cog;
      if (diff > 180.0) diff -= 360.0; 
      if (diff < -180.0) diff += 360.0;
      if (abs(diff) > 0.5) { 
        l_cog += diff * 0.2; 
        if (l_cog >= 360.0) l_cog -= 360.0; 
        if (l_cog < 0.0) l_cog += 360.0; 
      }
    }

    l_tgt_sog = calculateTargetSOG(l_cog, l_twd, l_tws, l_eng, l_rpm);
    l_sog += (l_tgt_sog - l_sog) * 0.05;

    float dist = (l_sog / 3600.0) * l_interval; 
    l_lat += (dist * cos(l_cog * PI / 180.0)) / 60.0;
    l_lon += (dist * sin(l_cog * PI / 180.0)) / (60.0 * cos(l_lat * PI / 180.0));

    l_total_log += dist;
    l_trip_log  += dist;

    // KROK 4: Zapis do Mutexa
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      true_wind_speed = l_tws;         true_wind_dir = l_twd;
      target_wind_speed = l_tgt_ws;    target_wind_dir = l_tgt_wd;
      current_depth = l_depth;         target_depth = l_tgt_depth;
      current_rpm = l_rpm;             
      current_cog = l_cog;             
      current_sog = l_sog;             target_sog = l_tgt_sog;
      current_lat = l_lat;             current_lon = l_lon;
      total_log_nm = l_total_log;      trip_log_nm = l_trip_log;
      xSemaphoreGive(dataMutex); 
    }

        // =================================================================
    // --- GENEROWANIE PAKIETU NMEA (zoptymalizowane + komentarze) ---
    // =================================================================
    float awa = 0.0, aws = 0.0; 
    calculateApparentWind(l_cog, l_sog, l_twd, l_tws, awa, aws);

    static char pack[2048];
    memset(pack, 0, sizeof(pack)); 
    int offset = 0;
    char baseLine[150], finalLine[160];
    char latB[20], lonB[20];
    
    sprintf(latB, "%02d%07.4f", (int)abs(l_lat), (abs(l_lat) - (int)abs(l_lat)) * 60.0);
    sprintf(lonB, "%03d%07.4f", (int)abs(l_lon), (abs(l_lon) - (int)abs(l_lon)) * 60.0);

    // ================================================================
    // === GRUPA FAST – wysyłana co sendInterval sekund (z WWW) ===
    // ================================================================

    // $GPRMC – Najważniejsza ramka pozycyjna (OpenCPN używa jej jako głównej)
    // Zawiera: czas, status, pozycja, SOG, COG, data, wariacja magnetyczna
    sprintf(baseLine, "$GPRMC,%s,A,%s,N,%s,E,%.1f,%.1f,%s,5.0,E", timeBuf, latB, lonB, l_sog, l_cog, dateBuf);
    addNMEAChecksumToLine(baseLine, finalLine); 
    offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

    // $GPGGA – Jakość fixu GPS (pasek satelitów + zielona/czerwona łódka)
    // Zawiera: czas, pozycja, jakość fixu, liczba satelitów, HDOP, wysokość
    sprintf(baseLine, "$GPGGA,%s,%s,N,%s,E,1,12,1.0,%.1f,M,0.0,M,,", timeBuf, latB, lonB, l_depth);
    addNMEAChecksumToLine(baseLine, finalLine);
    offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

    // $GPVTG – Kurs i prędkość (COG + SOG w węzłach i km/h)
    // OpenCPN bardzo lubi tę ramkę do wyświetlania wektora prędkości
    sprintf(baseLine, "$GPVTG,%.1f,T,%.1f,M,%.1f,N,%.1f,K", l_cog, l_cog, l_sog, l_sog * 1.852f);
    addNMEAChecksumToLine(baseLine, finalLine);
    offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

    // $IIHDT – Heading prawdziwy (kurs kompasu / gyro)
    // Z leeway (dryf wiatrowy) – heading ≠ COG jak w prawdziwym jachcie
    float l_twa = l_twd - l_cog;
    while (l_twa <= -180.0f) l_twa += 360.0f;
    while (l_twa > 180.0f)  l_twa -= 360.0f;
    float heading = l_cog + (l_twa > 0 ? -4.0f : 4.0f);
    if (heading < 0.0f)   heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;

    sprintf(baseLine, "$IIHDT,%.1f,T", heading);
    addNMEAChecksumToLine(baseLine, finalLine); 
    offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

    // $WIMWV – Wiatr pozorny (Apparent Wind) – najważniejszy dla żeglarzy
    // Kąt i prędkość wiatru względem jachtu
    //sprintf(baseLine, "$WIMWV,%.1f,R,%.1f,N,A", awa, aws);
    sprintf(baseLine, "$WIMWV,%.1f,R,%.1f,N,A", 0, 15); // staly wiatr dla testow 
    addNMEAChecksumToLine(baseLine, finalLine); 
    offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

    // $VHW – Prędkość przez wodę (Speed Through Water) + heading
    // Dashboard pokazuje różnicę STW vs SOG (prąd)
    sprintf(baseLine, "$VHW,%.1f,T,,M,%.1f,N,,K", l_cog, l_sog);
    addNMEAChecksumToLine(baseLine, finalLine);
    offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);


    // ================================================================
    // === GRUPA MEDIUM – co 2 × sendInterval ===
    // ================================================================
    static int packetCounter = 0;
    packetCounter++;

    if (packetCounter % 2 == 0) {

        // $SDDPT – Głębokość wody (Depth)
        sprintf(baseLine, "$SDDPT,%.1f,0.0", l_depth);
        addNMEAChecksumToLine(baseLine, finalLine); 
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

        // $WIMWD – Wiatr rzeczywisty (True Wind Direction + Speed)
        // Bardzo lubiany w instrumentach wiatrowych OpenCPN
        sprintf(baseLine, "$WIMWD,%.1f,T,%.1f,M,%.1f,N,%.1f,M", l_twd, l_twd, l_tws, l_tws * 0.5144f);
        addNMEAChecksumToLine(baseLine, finalLine);
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

        // $IIHDG – Heading magnetyczny + dewiacja + wariacja
        sprintf(baseLine, "$IIHDG,%.1f,,,%.1f,E", l_cog, 5.0);
        addNMEAChecksumToLine(baseLine, finalLine);
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);
    }


    // ================================================================
    // === GRUPA SLOW – co 5 × sendInterval ===
    // ================================================================
    if (packetCounter % 5 == 0) {

        // $IIVLW – Log (przebyta droga przez wodę) – total + trip
        sprintf(baseLine, "$IIVLW,%.1f,N,%.1f,N", l_total_log, l_trip_log);
        addNMEAChecksumToLine(baseLine, finalLine);
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

        // $IIXDR – Dane dodatkowe (tu temperatura silnika / inne)
        /*if (l_rpm > 0.0f) {
            sprintf(baseLine, "$IIXDR,T,%.1f,R,ENGINE#0", l_rpm);
        } else {
            sprintf(baseLine, "$IIXDR,T,0.0,R,ENGINE#0");
        }
        addNMEAChecksumToLine(baseLine, finalLine);
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine); */

        // $IIRPM – Obroty silnika (Engine RPM)
        sprintf(baseLine, "$IIRPM,E,1,%.0f,,A", l_rpm);
        addNMEAChecksumToLine(baseLine, finalLine);
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);

        // $GPZDA – Dokładny czas i data UTC (do synchronizacji OpenCPN)
        sprintf(baseLine, "$GPZDA,%s,%02d,%02d,%04d,,", 
                timeBuf, timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900);
        addNMEAChecksumToLine(baseLine, finalLine);
        offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);
    }

    if (packetCounter > 10000) packetCounter = 0;   // zabezpieczenie przed przepełnieniem

    // =================================================================
    // --- AIS CELE ---
    // =================================================================
    for(int i = 0; i < 3; i++) {
      unsigned long nowMs = millis();

      float dx = (l_lon - targets[i].lon) * cos(l_lat * PI / 180.0);
      float dy = l_lat - targets[i].lat;
      float dist_nm = sqrt(dx*dx + dy*dy) * 60.0;

      float bearing_to_jacht = atan2(dx, dy) * 180.0 / PI;
      if (bearing_to_jacht < 0) bearing_to_jacht += 360.0;

      bool borderRisk = (dist_nm > targets[i].max_radius_nm);
      unsigned long timeSinceLastDecision = nowMs - targets[i].lastCourseDecision;

      // --- RYBAK ---
      if (targets[i].id == 3) {
          if (borderRisk) targets[i].is_returning = true;
          else if (dist_nm <= 1.0) targets[i].is_returning = false;

          if (targets[i].is_returning) {
              targets[i].sog = targets[i].max_sog;
              if (timeSinceLastDecision > 5000) {
                  targets[i].target_cog = bearing_to_jacht;
                  targets[i].lastCourseDecision = nowMs;
                  if (debug_mode) Serial.printf("%s [AIS DEBUG] %s złapany na gumkę! Wraca na strefę.\n", getTimestamp().c_str(), targets[i].name);
              }
          } else {
              targets[i].sog = 0.0; 
          }
      } 
      // --- PROM / TANKOWIEC ---
      else {
          if (borderRisk) {
              // EFEKT GUMKI: Wypadli ze strefy (Złapani na smycz)
              targets[i].sog = targets[i].max_sog; 
              if (timeSinceLastDecision > 15000) {
                  targets[i].target_cog = bearing_to_jacht + random(-20, 21);
                  targets[i].planned_cog = targets[i].target_cog; 
                  targets[i].is_avoiding = false;                 
                  targets[i].lastCourseDecision = nowMs;
              }
          } else {
              // === 1. ANALIZA RYZYKA (DETEKCJA ARPA) ===
              // Sprawdzenie, czy jacht i statek AIS są na kursie kolizyjnym.
              bool arpa_alarm = checkCollisionRisk_ARPA(l_lat, l_lon, l_sog, l_cog,
                                                        targets[i].lat, targets[i].lon, targets[i].sog, targets[i].cog);
              
              // Określenie ról na podstawie MPZZM (COLREGs):
              // Statek ustępujący (Give-way) vs statek z pierwszeństwem (Stand-on).
              bool i_am_give_way = true; 
              float rel_bearing_jacht = bearing_to_jacht - targets[i].cog;
              while (rel_bearing_jacht > 180.0) rel_bearing_jacht -= 360.0;
              while (rel_bearing_jacht < -180.0) rel_bearing_jacht += 360.0;

              // Jeśli płyniemy na silniku, sprawdzamy kąty (sektory świateł), aby ustalić kto ustępuje.
              if (arpa_alarm && l_eng) {
                  if (rel_bearing_jacht < -10.0 || abs(rel_bearing_jacht) > 112.5) i_am_give_way = false; 
                  if (abs(rel_bearing_jacht) <= 10.0) i_am_give_way = true;
              }

              // Aktywacja trybu unikania (Avoidance)
              if (arpa_alarm) {
                  // Jeśli statek AIS musi ustąpić (np. jachtowi na żaglach), planuje manewr.
                  if (i_am_give_way && !targets[i].is_avoiding) {
                      targets[i].planned_cog = targets[i].target_cog; 
                      targets[i].is_avoiding = true;
                  } 
                  else if (!i_am_give_way) {
                      targets[i].lastCourseDecision = nowMs;
                  }
              }
              
              // === 2. WYKONYWANIE MANEWRU UNIKANIA ===
                if (targets[i].is_avoiding) {
                  // Sprawdzenie, czy zagrożenie minęło (Past and Clear)
                  bool isPastAndClear = (abs(rel_bearing_jacht) > 100.0 && dist_nm > 1.5) || (dist_nm > 3.0);
                  if (isPastAndClear) {
                    // Powrót na pierwotnie planowany kurs
                    targets[i].target_cog = targets[i].planned_cog;
                    if (targets[i].sog < targets[i].base_sog) {
                      targets[i].sog += 0.2;
                      if (l_dbg) Serial.printf("%s [AIS DEBUG] %s zwiększa prędkość (powrót do bazowej)\n", getTimestamp().c_str(), targets[i].name);
                    }

                    if (timeSinceLastDecision > 5000) targets[i].lastCourseDecision = nowMs;

                    float diffC = targets[i].target_cog - targets[i].cog;
                    while (diffC > 180.0) diffC -= 360.0;
                    while (diffC < -180.0) diffC += 360.0;
                    if (abs(diffC) < 2.0) {
                      targets[i].is_avoiding = false;
                      if (l_dbg) Serial.printf("%s [AIS DEBUG] %s zakończył manewr unikania\n", getTimestamp().c_str(), targets[i].name);
                    }
                  } else {
                    // TRWA MANEWR: Zmniejszenie prędkości i dynamiczna zmiana kursu.
                    if (targets[i].sog > (targets[i].base_sog * 0.4)) {
                      targets[i].sog -= 0.5;
                      if (l_dbg) Serial.printf("%s [AIS DEBUG] %s zmniejsza prędkość podczas unikania\n", getTimestamp().c_str(), targets[i].name);
                    }

                    // Obliczanie kąta przecięcia kursów, aby zdecydować o kierunku skrętu (ucieczka za rufę).
                    float crossing_angle = l_cog - targets[i].planned_cog;
                    while (crossing_angle > 180.0) crossing_angle -= 360.0;
                    while (crossing_angle < -180.0) crossing_angle += 360.0;

                    float ucieczka_kat = 60.0; 
                    if (crossing_angle > 10.0 && crossing_angle < 170.0) ucieczka_kat = -60.0; 

                    float nowy_kurs_ucieczki = targets[i].planned_cog + ucieczka_kat;
                    while (nowy_kurs_ucieczki >= 360.0) nowy_kurs_ucieczki -= 360.0;
                    while (nowy_kurs_ucieczki < 0.0) nowy_kurs_ucieczki += 360.0;

                    float roznica = nowy_kurs_ucieczki - targets[i].cog;
                    while (roznica > 180.0) roznica -= 360.0;
                    while (roznica < -180.0) roznica += 360.0;

                    if (abs(targets[i].target_cog - nowy_kurs_ucieczki) > 1.0) {
                      targets[i].target_cog = nowy_kurs_ucieczki; 
                      targets[i].lastCourseDecision = nowMs;
                      if (l_dbg) Serial.printf("%s [AIS DEBUG] %s unika: skręt w %s\n", getTimestamp().c_str(), targets[i].name, roznica > 0 ? "prawo" : "lewo");
                    } else {
                      if (l_dbg) Serial.printf("%s [AIS DEBUG] %s unika: nie zmienia kursu (utrzymuje)\n", getTimestamp().c_str(), targets[i].name);
                    }
                  }
                } else {
                  if (targets[i].sog > targets[i].base_sog) targets[i].sog -= 0.5;
                  if (targets[i].sog < targets[i].base_sog) targets[i].sog = targets[i].base_sog;

                  if (timeSinceLastDecision > 90000) {
                      targets[i].target_cog = targets[i].cog + random(-45, 46);
                      targets[i].planned_cog = targets[i].target_cog; 
                      targets[i].lastCourseDecision = nowMs;
                  }
              }
          }
      }

      float diff = targets[i].target_cog - targets[i].cog;
      if (diff > 180.0) diff -= 360.0;
      if (diff < -180.0) diff += 360.0;

      float maxTurnPerSecond = targets[i].max_turn_rate; 
      float turnThisCycle = constrain(diff, -maxTurnPerSecond, maxTurnPerSecond) * l_interval;
      targets[i].cog += turnThisCycle;

      float d = (targets[i].sog / 3600.0) * l_interval;
      targets[i].lat += (d * cos(targets[i].cog * PI / 180.0)) / 60.0;
      targets[i].lon += (d * sin(targets[i].cog * PI / 180.0)) / (60.0 * cos(targets[i].lat * PI / 180.0));

      int mmsi = 261000000 + targets[i].id; 
      float current_hdg = targets[i].cog; 
      if (targets[i].id == 3 && targets[i].sog < 0.5) current_hdg = 511.0; 

      generateAIVDM(mmsi, targets[i].lat, targets[i].lon, targets[i].sog, targets[i].cog, current_hdg, finalLine);
      offset += snprintf(pack + offset, sizeof(pack) - offset, "%s", finalLine);
    }
    
    // =================================================================
    // --- WYSYŁANIE DANYCH ---
    // =================================================================
    if (l_udp) { 
      udp.beginPacket(udpAddress, port); 
      udp.print(pack); 
      udp.endPacket(); 
    }
    
    if (l_tcp) { 
      for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) {
          tcpClients[i].print(pack); 
        }
      }
    }
    
    char wsJson[150];
    snprintf(wsJson, sizeof(wsJson), 
        "{\"c\":%.1f,\"s\":%.1f,\"ts\":%.1f,\"n\":%d,\"e\":%d,\"r\":%d}",
        l_cog, l_sog, l_tgt_sog, l_nav ? 1 : 0, l_eng ? 1 : 0, (int)l_rpm);
    ws.textAll(wsJson);

    // (Wysyłanie do Serial tylko w trybie debug)
    if (l_dbg) {
      Serial.println("\n------------------------------------------------");
      Serial.printf("%s [FIZYKA] SOG: %.1f kn (Cel: %.1f kn) | Silnik: %s (%.0f RPM)\n", getTimestamp().c_str(), l_sog, l_tgt_sog, l_eng?"Wlaczony":"Wylaczony", l_rpm);
      Serial.printf("%s [METEO]  TWS: %.1f kn z %.0f st. | AWS: %.1f kn (Kąt: %.1f st.)\n", getTimestamp().c_str(), l_tws, l_twd, aws, awa);
      Serial.printf("%s --- PAKIET NMEA WYSŁANY DO SIECI ---\n", getTimestamp().c_str());
      //Serial.print(pack); // niepotrzebne, bo poniżej każda linia z timestampem
      String ts = getTimestamp();
      char* line = pack;
      while (*line) {
        char* next = strchr(line, '\n');
        if (next) {
          *next = 0;
        }
        if (strlen(line) > 0) {
          Serial.print(ts);
          Serial.print(" ");
          Serial.println(line);
        }
        if (!next) break;
        line = next + 1;
      }
    }
  } 
  else {
    vTaskDelay(pdMS_TO_TICKS(2));
  }

    // === 5. Blok statyczny AIS (co 2 minuty) ===
    if (millis() - lastStaticAISTick >= 60000) {          // <-- co minutę
    lastStaticAISTick = millis();
    
    bool l_udp_stat = false, l_tcp_stat = false, l_dbg_stat = false;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        l_udp_stat = enableUDP;
        l_tcp_stat = enableTCP;
        l_dbg_stat = debug_mode;
        xSemaphoreGive(dataMutex);
    }

    char staticPack[700] = {0};   // powiększony bufor
    int offset = 0;

    static bool sendPartB = false;        // flaga na zmianę A / B
    sendPartB = !sendPartB;               // za każdym razem przełącza

    if (sendPartB) {
        // Wysyłamy Part B (callsign + typ + wymiary)
        offset += snprintf(staticPack + offset, sizeof(staticPack) - offset, "%s", staticAIS_PromB);
        offset += snprintf(staticPack + offset, sizeof(staticPack) - offset, "%s", staticAIS_TankowiecB);
        offset += snprintf(staticPack + offset, sizeof(staticPack) - offset, "%s", staticAIS_RybakB);
        
        if (l_dbg_stat) Serial.println(">>> [AIS] Wysłano Type 24B (Part B)");
    } 
    else {
        // Wysyłamy Part A (tylko nazwa)
        offset += snprintf(staticPack + offset, sizeof(staticPack) - offset, "%s", staticAIS_PromA);
        offset += snprintf(staticPack + offset, sizeof(staticPack) - offset, "%s", staticAIS_TankowiecA);
        offset += snprintf(staticPack + offset, sizeof(staticPack) - offset, "%s", staticAIS_RybakA);
        
        if (l_dbg_stat) Serial.println(">>> [AIS] Wysłano Type 24A (Part A)");
    }

    // Wysyłanie przez UDP / TCP
    if (l_udp_stat) { 
        udp.beginPacket(udpAddress, port); 
        udp.print(staticPack); 
        udp.endPacket(); 
    }
    
    if (l_tcp_stat) { 
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (tcpClients[i] && tcpClients[i].connected()) {
                tcpClients[i].print(staticPack); 
            }
        }
    }
}


  esp_task_wdt_reset(); 
}
// END OF FILE