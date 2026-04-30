
#include <Arduino.h>
#include "runSkipperAI.h"
#include "config.h"

void runSkipperAI(const YachtState &snap)
{ // Funkcja symulująca inteligencję skippera
  const float side_deg = 0.25;
  const float start_lat = 55.5000;
  const float start_lon = 18.0000;

  // Definicja wierzchołków kwadratu
  float wp_lat[4] = {start_lat + side_deg, start_lat + side_deg, start_lat, start_lat};
  float wp_lon[4] = {start_lon, start_lon + side_deg, start_lon + side_deg, start_lon};

  // 1. Obliczanie namiaru (bearing) do aktualnego punktu drogi
  float dy = wp_lat[snap.currentWaypoint] - snap.lat;
  float dx = (wp_lon[snap.currentWaypoint] - snap.lon) * cosf(snap.lat * PI / 180.0);
  float bearing = atan2(dx, dy) * 180.0 / PI;
  if (bearing < 0)
    bearing += 360.0;

  // 2. Obliczanie dystansu do celu
  float dist_nm = sqrtf(dx * dx + dy * dy) * 60.0;

  // Przygotowanie nowej wartości WP (jeśli osiągnięto cel)
  int nextWaypoint = snap.currentWaypoint;
  if (dist_nm < 0.5) {
      nextWaypoint = (snap.currentWaypoint + 1) % 4;
      Serial.printf(">>> [SKIPPER] Osiągnięto Waypoint! Nowy cel: %d\n", nextWaypoint);
  }

  // 3. Logika zarządzania napędem (Silnik vs Żagle)
  // Obliczamy kąt wiatru względem kursu, który Skipper CHCE obrać
  float target_twa = snap.twd - bearing;
  while (target_twa <= -180.0)
    target_twa += 360.0;
  while (target_twa > 180.0)
    target_twa -= 360.0;
  float abs_twa = fabsf(target_twa);

  bool should_use_engine = false;

  // Zasada: Jeśli wiatr w dziób (kąt < 35 stopni) - żagle nie działają, włącz silnik
  if (abs_twa < 35.0)
  {
    should_use_engine = true;
    if (!snap.engine_on)
      Serial.println(">>> [SKIPPER] Wiatr w dziób! Odpalam silnik.");
  }
  // Zasada: Jeśli flauta (wiatr < 4 węzły) - włącz silnik
  else if (snap.tws < 4.0)
  {
    should_use_engine = true;
    if (!snap.engine_on)
      Serial.println(">>> [SKIPPER] Brak wiatru! Odpalam silnik.");
  }
  // Zasada: Jeśli warunki są dobre - gasi silnik i żeglujemy
  else if (snap.engine_on && abs_twa > 45.0 && snap.tws > 6.0)
  {
    should_use_engine = false;
    Serial.println(">>> [SKIPPER] Dobre warunki! Gaszę silnik, stawiam żagle.");
  }
  else
  {
    should_use_engine = snap.engine_on; // Zachowaj obecny stan
  }

  // 4. Aktualizacja stanu w Mutexie
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)))
  {
    yacht.navMode = true;
    yacht.skipperActive = true;
    yacht.target_nav_cog = bearing;
    yacht.currentWaypoint = nextWaypoint;

    if (should_use_engine != yacht.engine_on)
    {
      yacht.engine_on = should_use_engine;
      if (yacht.engine_on)
      {
        yacht.target_rpm = 2200.0;
        yacht.rpm = 2200.0;
      }
      else
      {
        yacht.target_rpm = 0.0;
        yacht.rpm = 0.0;
      }
    }
    xSemaphoreGive(dataMutex);
  }
  else
  {
    if (config.log_level >= 1)
    {
      Serial.println("[SKIPPER] Timeout mutex przy zapisie - pomijam aktualizację");
    }
  }

  // Logowanie co jakiś czas dla debugu
  static unsigned long lastSkipperLog = 0;
  if (millis() - lastSkipperLog > 10000)
  {
    Serial.printf(">>> [SKIPPER] Cel: WP%d | Dist: %.1f nm | Brg: %.0f | TWA: %.0f | Eng: %s\n",
                  snap.currentWaypoint, dist_nm, bearing, target_twa, should_use_engine ? "ON" : "OFF");
    lastSkipperLog = millis();
  }
}
