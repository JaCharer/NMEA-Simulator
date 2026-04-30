#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>
#include "SignalK.h"
#include "config.h"
#include "WebServer.h"


TaskHandle_t SignalKTaskHandle;

void signalKTaskCode(void *pvParameters)
{
  for (;;)
  {
    if (config.enableSignalK)
    {
      // 1. Błyskawiczny snapshot danych (Mutex na ułamek sekundy)
      YachtState snap;
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
      {
        snap = yacht;
        xSemaphoreGive(dataMutex);

        // 2. Ciężka robota: budowanie i wysyłka JSON-a (poza Mutexem!)
        sendSignalKData(snap);
      }
    }
    // 3. Odpoczynek zadania - 500 ms (czyli mamy 2 Hz)
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void sendSignalKData(const YachtState &snap)
{
  float sog_ms = snap.sog * 0.514444, cog_rad = snap.cog * PI / 180.0;
  float tws_ms = snap.tws * 0.514444, twd_rad = snap.twd * PI / 180.0;
  float heading_rad = snap.heading * PI / 180.0;

  struct tm timeinfo;
  char isoTime[30];
  if (getLocalTime(&timeinfo, 10))
    strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);
  else
    strcpy(isoTime, "1970-01-01T00:00:00.000Z");

  JsonDocument doc;
  doc["context"] = "vessels.urn:mrn:imo:mmsi:265599691";
  JsonArray updates = doc["updates"].to<JsonArray>();
  JsonObject update = updates.add<JsonObject>();
  update["source"]["label"] = "NMEA Simulator";
  update["source"]["type"] = "NMEA0183";
  update["timestamp"] = isoTime;
  JsonArray values = update["values"].to<JsonArray>();

  auto addVal = [&](const char *path, double value)
  {
    JsonObject v = values.add<JsonObject>();
    v["path"] = path;
    v["value"] = value;
  };

  JsonObject pos = values.add<JsonObject>();
  pos["path"] = "navigation.position";
  pos["value"]["latitude"] = snap.lat;
  pos["value"]["longitude"] = snap.lon;

  addVal("navigation.courseOverGroundTrue", cog_rad);
  addVal("navigation.headingTrue", heading_rad);
  addVal("navigation.speedOverGround", sog_ms);
  addVal("environment.depth.belowTransducer", snap.depth);
  addVal("environment.wind.speedTrue", tws_ms);
  addVal("environment.wind.directionTrue", twd_rad);
  addVal("environment.wind.speedApparent", snap.aws * 0.514444);
  addVal("environment.wind.angleApparent", snap.awa * PI / 180.0);
  addVal("propulsion.port.revolutions", snap.rpm / 60.0);

  static char jsonBuffer[1024];
  serializeJson(doc, jsonBuffer);
  skWs.textAll(jsonBuffer);
}
