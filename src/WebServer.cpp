
#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include "WebServer.h"


// WebServer webServer(80);
AsyncWebServer webServer(80);
AsyncWebSocket ws("/ws");                  // WebSocket dla Twojej strony WWW
AsyncWebSocket skWs("/signalk/v1/stream"); // Dedykowany WebSocket Signal K


// =========================================================================
// OBSŁUGA ZDARZEŃ WEBSOCKET (KOMENDY Z PRZEGLĄDARKI)
// =========================================================================

// --- WSPÓLNY PARSER KOMEND DLA OBU WEBSOCKETÓW ---
void handleIncomingData(uint8_t *data, size_t len)
{
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)))
  {
    if (len > 0 && data[0] == '{')
    {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, data, len);
      if (!error)
      {

        // Pomocnicza lambda do przetwarzania konkretnych par path/value
        auto processEntry = [&](const char *path, JsonVariant val)
        {
          if (!path)
            return;

          // 1. Kurs autopilota (tylko jeśli tryb NAV jest aktywny)
          if (strcmp(path, "steering.autopilot.target.headingTrue") == 0 ||
              strcmp(path, "navigation.courseRhumbline.nextPoint.bearingTrue") == 0)
          {
            // ZMIANA: Zezwalamy na zmianę kursu z zewnątrz TYLKO gdy Skipper AI śpi
            if (yacht.navMode && !yacht.skipperActive)
            {
              yacht.target_nav_cog = val.as<float>() * (180.0 / PI);
              // USUNIĘTO: yacht.skipperActive = false;
              if (config.log_level >= 2)
                Serial.printf(">>> [NAV-ACTIVE] Kurs z OpenCPN: %.1f\n", yacht.target_nav_cog);
            }
            else if (yacht.navMode && yacht.skipperActive && config.log_level >= 3)
            {
              Serial.println(">>> [BLOKADA] Ignoruję OpenCPN - Skipper AI trzyma ster!");
            }
          }

          // 2. Przełącznik stanu Autopilota (zawsze dostępny)
          else if (strcmp(path, "steering.autopilot.state") == 0)
          {
            yacht.skipperActive = false; // Zewnętrzna zmiana stanu AP zawsze ucisza Skipper AI
            if (val.is<bool>())
            {
              yacht.navMode = val.as<bool>();
            }
            else
            {
              const char *state = val.as<const char *>();
              if (state != nullptr)
              {
                yacht.navMode = (strcmp(state, "engaged") == 0 || strcmp(state, "auto") == 0);
              }
            }
          }
          // 3. Sterowanie silnikiem
          else if (strcmp(path, "propulsion.main.state") == 0)
          {
            yacht.engine_on = val.as<bool>();
            yacht.rpm = yacht.engine_on ? yacht.target_rpm : 0.0;
          }
          else if (strcmp(path, "propulsion.main.revolutions") == 0)
          {
            yacht.target_rpm = val.as<float>() * 60.0;
            if (yacht.engine_on)
              yacht.rpm = yacht.target_rpm;
          }
        };

        // Sprawdzamy czy to format Delta (tablica updates)
        JsonVariant updates = doc["updates"];
        if (updates.is<JsonArray>())
        {
          for (JsonObject update : updates.as<JsonArray>())
          {
            JsonVariant values = update["values"];
            if (values.is<JsonArray>())
            {
              for (JsonObject vObj : values.as<JsonArray>())
              {
                processEntry(vObj["path"], vObj["value"]);
              }
            }
          }
        }
        // Sprawdzamy czy to format płaski (klucz path)
        else if (doc["path"].is<const char *>())
        {
          processEntry(doc["path"], doc["value"]);
        }
      }
    }
    else
    {
      char cmd[32];
      int copyLen = len < 31 ? len : 31;
      memcpy(cmd, data, copyLen);
      cmd[copyLen] = '\0'; // Zamykamy tekst zerem (bardzo ważne!)

      // Zamiast == używamy standardowego strcmp (String Compare)
      if (strcmp(cmd, "NAV") == 0)
      {
        yacht.navMode = !yacht.navMode;
        yacht.skipperActive = false;
      }
      else if (strcmp(cmd, "L1") == 0)
      {
        yacht.navMode = false; // Ręczna zmiana wyłącza Navi
        yacht.cog -= 1.0;
        yacht.skipperActive = false;
      }
      else if (strcmp(cmd, "R1") == 0)
      {
        yacht.navMode = false; // Ręczna zmiana wyłącza Navi
        yacht.cog += 1.0;
        yacht.skipperActive = false;
      }
      else if (strcmp(cmd, "L10") == 0)
      {
        yacht.navMode = false; // Ręczna zmiana wyłącza Navi
        yacht.cog -= 10.0;
        yacht.skipperActive = false;
      }
      else if (strcmp(cmd, "R10") == 0)
      {
        yacht.navMode = false; // Ręczna zmiana wyłącza Navi
        yacht.cog += 10.0;
        yacht.skipperActive = false;
      }
      // Zamiast startsWith("ENG:")
      else if (strncmp(cmd, "ENG:", 4) == 0)
      {
        int e_state = 0;
        float e_rpm = 0.0;
        // MAGIA C++: sscanf automatycznie wyciągnie liczby z tekstu typu "ENG:1:2500"
        if (sscanf(cmd, "ENG:%d:%f", &e_state, &e_rpm) == 2)
        {
          yacht.engine_on = (e_state == 1);
          yacht.target_rpm = e_rpm;
          yacht.rpm = yacht.engine_on ? yacht.target_rpm : 0.0;
        }
      }
      if (yacht.cog < 0.0)
        yacht.cog += 360.0;
      if (yacht.cog >= 360.0)
        yacht.cog -= 360.0;
    }
    xSemaphoreGive(dataMutex);
  }
  else
  {
    if (config.log_level >= 1)
    {
      Serial.println("[WEB/SK/TCP] Timeout mutex przy komendzie - odrzucam ramkę");
    }
    return;
  }
}

// --- WebSocket EVENT HANDLER DLA STRONY WWW ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    if (config.log_level >= 1)
    {
      Serial.println("[WS] Przegladarka polaczona");
    }
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(80)))
    {
      YachtState snap = yacht;
      xSemaphoreGive(dataMutex);
      JsonDocument doc;
      doc["c"] = snap.cog;
      doc["s"] = snap.sog;
      doc["ts"] = snap.target_sog;
      doc["n"] = snap.navMode ? 1 : 0;
      doc["e"] = snap.engine_on ? 1 : 0;
      doc["r"] = (int)snap.rpm;
      doc["sai"] = snap.skipperActive ? 1 : 0;
      doc["sk"] = config.enableSignalK ? 1 : 0;
      doc["tws"] = snap.tws;
      doc["twd"] = snap.twd;

      char initBrowserBuffer[256];
      memset(initBrowserBuffer, 0, sizeof(initBrowserBuffer));
      serializeJson(doc, initBrowserBuffer, sizeof(initBrowserBuffer));
      client->text(initBrowserBuffer);
    }
    else
    {
      if (config.log_level >= 1)
      {
        Serial.println("[WS] Timeout mutex przy CONNECT - pomijam wysyłanie init");
      }
    }
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_TEXT)
    {
      if (config.log_level >= 3)
      {
        Serial.printf("[WEB-IN] Komenda ze strony: %.*s\n", len, (char *)data);
      }
      handleIncomingData(data, len);
    }
  }
}

// --- WebSocket EVENT HANDLER DLA SIGNAL K ---
void onSkWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    // client->text("{\"self\":\"vessels.self\",\"version\":\"1.7.0\"}");
    client->text("{\"roles\":[\"master\",\"main\"],\"self\":\"vessels.urn:mrn:imo:mmsi:265599691\",\"version\":\"1.7.0\"}");
    if (config.log_level >= 1)
    {
      Serial.println("[SK WS] Klient Signal K polaczony");
    }
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_TEXT)
    {
      if (config.log_level >= 3)
      {
        Serial.printf("[SK-IN] Otrzymano od OpenCPN: %.*s\n", len, (char *)data);
      }
      handleIncomingData(data, len);
    }
  }
}


// =========================================================================
// WEB SERVER – ASYNCHRONICZNA WERSJA (stabilna 24/7+)
// =========================================================================
void setupWebServer()
{
  // === WEBSOCKET ===
  ws.onEvent(onWsEvent);
  webServer.addHandler(&ws);
  skWs.onEvent(onSkWsEvent);
  webServer.addHandler(&skWs);

  // === STRONY HTML (Ładowane z pamięci LittleFS) ===
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/index.html", "text/html"); });

  webServer.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/config.html", "text/html"); });

  // === OBSŁUGA PREFLIGHT CORS DLA WEBSOCKET (dla OpenCPN 5.12) ===
  webServer.on("/ws", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
               {
    AsyncWebServerResponse *response = request->beginResponse(204);
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
    request->send(response);
    if (config.log_level >= 3) {
        Serial.println("[CORS] OPTIONS /ws obsłużone");
    } });

  webServer.on("/signalk", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    // 1. Pobieramy surowy typ IPAddress (zero Stringów!)
    IPAddress ip = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();

    char discoveryBuffer[512];
    memset(discoveryBuffer, 0, sizeof(discoveryBuffer));

    // 2. Magia snprintf: formatujemy IP "w locie" używając %u.%u.%u.%u
    snprintf(discoveryBuffer, sizeof(discoveryBuffer),
      "{\n"
      "  \"self\": \"vessels.urn:mrn:imo:mmsi:265599691\",\n"
      "  \"endpoints\": {\n"
      "    \"v1\": {\n"
      "      \"version\": \"1.7.0\",\n"
      "      \"signalk-ws\": \"ws://%u.%u.%u.%u/signalk/v1/stream\",\n"
      "      \"signalk-http\": \"http://%u.%u.%u.%u/signalk\"\n"
      "    }\n"
      "  },\n"
      "  \"server\": {\n"
      "    \"id\": \"esp32-nmea-sim\",\n"
      "    \"version\": \"2.2\",\n"
      "    \"name\": \"Jacht Symulator NMEA/AIS\"\n"
      "  }\n"
      "}",
      ip[0], ip[1], ip[2], ip[3], // Pierwsze wstawienie IP (dla ws://)
      ip[0], ip[1], ip[2], ip[3]  // Drugie wstawienie IP (dla http://)
    );

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", discoveryBuffer);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);

    if (config.log_level >= 1) {
        Serial.println("[SignalK] Discovery wysłane poprawnie");
    } });

  // --- OBSŁUGA POLECEŃ STEROWANIA PRZEZ HTTP PUT (Standard OpenCPN / Signal K) ---
  webServer.on("/signalk/v1/api/vessels/self/steering/autopilot/target/headingTrue", HTTP_PUT, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
               {
        
        // DODANO TEN WARUNEK: Sprawdzamy czy to już koniec paczki danych
        if (index + len == total) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
                if (yacht.navMode) {
                    // ZMIANA: Sprawdzamy czy AI nie ma wyższego priorytetu
                    if (yacht.skipperActive) {
                        request->send(403, "text/plain", "Forbidden - Skipper AI has control");
                        if (config.log_level >= 2) Serial.println(">>> [HTTP-PUT] Odrzucono kurs z OpenCPN (Skipper AI aktywny)");
                    } else {
                        JsonDocument doc;
                        DeserializationError error = deserializeJson(doc, data, len);
                        
                        if (!error && doc["value"].is<float>()) {
                            float headingRad = doc["value"];
                            yacht.target_nav_cog = headingRad * (180.0 / PI);
                            // USUNIĘTO: yacht.skipperActive = false; 
                            
                            if (config.log_level >= 1) {
                                Serial.printf(">>> [HTTP-PUT] OpenCPN ustawia kurs: %.1f\n", yacht.target_nav_cog);
                            }
                            request->send(200, "application/json", "{\"state\":\"COMPLETED\"}");
                        } else {
                            request->send(400, "text/plain", "Bad Request - Need value");
                        }
                    }
                } else {
                    request->send(403, "text/plain", "Autopilot is Standby");
                }
                xSemaphoreGive(dataMutex);
            } else {
                if (config.log_level >= 1) {
                    Serial.println("[HTTP-PUT] Ominięto klatkę - Mutex zablokowany!");
                }
                request->send(503, "text/plain", "Service Unavailable - Try Again");
            }
        } });

  webServer.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    int n = WiFi.scanNetworks();
    
    // Tworzymy strumień prosto do klienta sieciowego
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for(int i = 0; i < n; i++) {
        JsonObject network = arr.add<JsonObject>();
        network["ssid"] = WiFi.SSID(i);
        network["rssi"] = WiFi.RSSI(i);
    }
    
    // Serializacja uderza od razu do strumienia sieciowego. Zero buforów i alokacji!
    serializeJson(doc, *response);
    request->send(response); });

  // === ZAPIS KONFIGURACJI WIFI (POST JSON) ===
  webServer.on("/saveconfig", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
               {
      if (index + len == total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (error) { request->send(400, "text/plain", "Bledny JSON"); return; }

        String ssid_new = doc["ssid"] | "";
        String pass_new = doc["pass"] | "";
        bool   ap_new   = doc["ap"] | false;

        bool wifiChanged = false;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
          if (ap_new != config.useAP_mode || 
              (ssid_new.length() > 0 && ssid_new != config.wifi_ssid) ||
              (ssid_new.length() > 0 && pass_new != config.wifi_pass)) {
            wifiChanged = true;
          }

          preferences.putString("wifi_ssid", ssid_new);
          preferences.putString("wifi_pass", pass_new);
          preferences.putBool("useAP", ap_new);

          config.useAP_mode = ap_new;
          if (ssid_new.length() > 0) config.wifi_ssid  = ssid_new;
          if (pass_new.length() > 0) config.wifi_pass  = pass_new;

          xSemaphoreGive(dataMutex);
          
        }else {
          if (config.log_level >= 1) {
          Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
          }
          request->send(503, "text/plain", "Service Unavailable - Try Again");
          return;
        }

        if (wifiChanged) {
          if (config.log_level >= 1) {
            Serial.println("[CONFIG] Zmiana ustawień WiFi → restart");
          }
          request->send(200, "text/plain", "Zapisano WiFi! ESP32 się restartuje...");
          delay(2000);
          ESP.restart();
        } else {
          request->send(200, "text/plain", "Zapisano konfiguracje (bez zmian sieciowych)");
        }
      } });

  // === BŁYSKAWICZNY ZAPIS USTAWIEŃ (TŁO) ===
  webServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request)
               {
                 if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
                 {
                   if (request->hasParam("u"))
                   {
                     config.enableUDP = request->getParam("u")->value() == "1";
                     preferences.putBool("udp", config.enableUDP);
                   }
                   if (request->hasParam("t"))
                   {
                     config.enableTCP = request->getParam("t")->value() == "1";
                     preferences.putBool("tcp", config.enableTCP);
                   }
                   if (request->hasParam("d"))
                   {
                     config.log_level = request->getParam("d")->value().toInt();
                     preferences.putInt("dbg", config.log_level);
                   }
                   if (request->hasParam("sk"))
                   { // NOWE: Opcja SignalK
                     config.enableSignalK = request->getParam("sk")->value() == "1";
                     preferences.putBool("sk", config.enableSignalK);
                   }
                   xSemaphoreGive(dataMutex);
                   request->send(200);
                 }
                 else
                 {
                   if (config.log_level >= 1)
                   {
                     Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
                   }
                   request->send(503, "text/plain", "Service Unavailable - Try Again");
                 } });

  // === SKIPPER AI TOGGLE ===
  webServer.on("/skipper", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
      yacht.skipperActive = !yacht.skipperActive;
      if (!yacht.skipperActive) yacht.navMode = false;
      bool currentState = yacht.skipperActive;
      xSemaphoreGive(dataMutex);

      request->send(200, "text/plain", currentState ? "Skipper ON" : "Skipper OFF");
    } else {
      if (config.log_level >= 1) Serial.println("[SYSTEM] Ominięto klatkę /skipper - Mutex zablokowany!");
      request->send(503, "text/plain", "Service Unavailable");
    } });

  // === ENDPOINT /data ===
  webServer.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
               {
                 // Kopiujemy cały stan jachtu i konfigurację w jednym bloku mutexa
                 YachtState snap;
                 bool l_udp, l_tcp, l_sk;

                 if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
                 {
                   snap = yacht;
                   l_udp = config.enableUDP;
                   l_tcp = config.enableTCP;
                   l_sk = config.enableSignalK;
                   xSemaphoreGive(dataMutex);
                   JsonDocument doc;
                   doc["cog"] = snap.cog;
                   doc["sog"] = snap.sog;
                   doc["tsog"] = snap.target_sog;
                   doc["lat"] = snap.lat;
                   doc["lon"] = snap.lon;
                   doc["dep"] = snap.depth;
                   doc["n"] = snap.navMode ? 1 : 0;
                   doc["e"] = snap.engine_on ? 1 : 0;
                   doc["sai"] = snap.skipperActive ? 1 : 0; // Skipper AI
                   doc["sk"] = l_sk ? 1 : 0;                // SignalK
                   doc["wp"] = snap.currentWaypoint;        // Nowe pole w API
                   doc["rpm"] = (int)snap.rpm;
                   doc["trpm"] = (int)snap.target_rpm;
                   doc["u"] = l_udp;
                   doc["t"] = l_tcp;
                   doc["d"] = config.log_level;
                   doc["tws"] = snap.tws;
                   doc["twd"] = snap.twd;

                   char output[256];
                   memset(output, 0, sizeof(output));
                   serializeJson(doc, output);
                   request->send(200, "application/json", output);
                 }
                 else
                 {
                   if (config.log_level >= 1)
                   {
                     Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
                   }
                   request->send(503, "text/plain", "Service Unavailable - Try Again");
                 } });

  // === ZMIANA KURSU ===
  webServer.on("/l10", HTTP_GET, [](AsyncWebServerRequest *request)
               {
                 if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
                 {
                   yacht.navMode = false; // Manualny reset Navi
                   yacht.cog -= 10.0;
                   if (yacht.cog < 0.0)
                     yacht.cog += 360.0;
                   yacht.skipperActive = false;
                   xSemaphoreGive(dataMutex);
                   request->send(200);
                 }
                 else
                 {
                   if (config.log_level >= 1)
                   {
                     Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
                   }
                   request->send(503, "text/plain", "Service Unavailable - Try Again");
                 } });

  webServer.on("/l1", HTTP_GET, [](AsyncWebServerRequest *request)
               {
                 if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
                 {
                   yacht.navMode = false; // Manualny reset Navi
                   yacht.cog -= 1.0;
                   if (yacht.cog < 0.0)
                     yacht.cog += 360.0;
                   yacht.skipperActive = false;
                   xSemaphoreGive(dataMutex);
                   request->send(200);
                 }
                 else
                 {
                   if (config.log_level >= 1)
                   {
                     Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
                   }
                   request->send(503, "text/plain", "Service Unavailable - Try Again");
                 } });
  webServer.on("/r1", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) { 
      yacht.navMode = false; yacht.skipperActive = false; // Manualny reset
      yacht.cog += 1.0;
      if (yacht.cog >= 360.0) yacht.cog -= 360.0;
      xSemaphoreGive(dataMutex); 
      request->send(200);
    } else {
      if (config.log_level >= 1) {
        Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
      }
      request->send(503, "text/plain", "Service Unavailable - Try Again");
    } });
  webServer.on("/r10", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) { 
      yacht.navMode = false; yacht.skipperActive = false; // Manualny reset
      yacht.cog += 10.0;
      if (yacht.cog >= 360.0) yacht.cog -= 360.0;
      xSemaphoreGive(dataMutex); 
      request->send(200);
    } else {
      if (config.log_level >= 1) {
        Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
      }
      request->send(503, "text/plain", "Service Unavailable - Try Again");
    } });

  // === NAV / AUTOPILOT ===
  webServer.on("/nav", HTTP_GET, [](AsyncWebServerRequest *request)
               { 
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) { 
      yacht.navMode = !yacht.navMode;
      yacht.skipperActive = false; // Ręczne przełączenie trybu wyłącza AI
      xSemaphoreGive(dataMutex); 
      request->send(200); 
    } else {
      if (config.log_level >= 1) {
        Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
      }
      request->send(503, "text/plain", "Service Unavailable - Try Again");
    } });

  // === SILNIK ===
  webServer.on("/engine", HTTP_GET, [](AsyncWebServerRequest *request)
               {
                 if (request->hasParam("s") && request->hasParam("r"))
                 {
                   if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
                   {
                     yacht.engine_on = request->getParam("s")->value() == "1";
                     yacht.target_rpm = request->getParam("r")->value().toFloat();

                     // BŁYSKAWICZNA AKTUALIZACJA: zapobiega "migotaniu" starych danych w przeglądarce
                     if (yacht.engine_on)
                     {
                       yacht.rpm = yacht.target_rpm;
                     }
                     else
                     {
                       yacht.rpm = 0.0;
                     }

                     xSemaphoreGive(dataMutex);
                     request->send(200);
                   }
                   else
                   {
                     if (config.log_level >= 1)
                     {
                       Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
                     }
                     request->send(503, "text/plain", "Service Unavailable - Try Again");
                   }
                 } });

  webServer.onNotFound([](AsyncWebServerRequest *request)
                       {
    if (config.log_level >= 3) {
        Serial.printf("[DEBUG-404] Metoda: %s | URL: %s\n", request->methodToString(), request->url().c_str());
    }
    request->send(404, "text/plain", "Not Found"); });

  webServer.begin(); // <<< WAŻNE – start asynchronicznego serwera
  Serial.println("[WEB] AsyncWebServer uruchomiony na porcie 80");
}

// =========================================================================
// Wysyła aktualny stan jachtu w formacie JSON przez WebSocket do dashboardu WWW
// =========================================================================
void updateBrowserDashboard(const YachtState &snap)
{
  if (ws.count() == 0)
    return;
  JsonDocument doc;
  doc["c"] = snap.cog;
  doc["s"] = snap.sog;
  doc["ts"] = snap.target_sog;
  doc["n"] = snap.navMode ? 1 : 0;
  doc["e"] = snap.engine_on ? 1 : 0;
  doc["r"] = (int)snap.rpm;
  doc["sai"] = snap.skipperActive ? 1 : 0;
  doc["sk"] = config.enableSignalK ? 1 : 0;
  doc["tws"] = snap.tws;
  doc["twd"] = snap.twd;
  doc["awa"] = snap.awa; // Nowe pola dla Dashboardu
  doc["aws"] = snap.aws;

  char buffer[256];
  serializeJson(doc, buffer);
  ws.textAll(buffer);
}