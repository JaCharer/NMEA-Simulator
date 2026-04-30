

#include "yacht_physics.h"


// =========================================================================
// REALISTYCZNA FIZYKA JACHTU (Prędkość kadłuba ~9.5 kn)
// =========================================================================
float calculateTargetSOG(float cog, float twd, float tws, bool eng_on, float rpm)
{
  float twa = twd - cog;
  while (twa <= -180.0)
    twa += 360.0;
  while (twa > 180.0)
    twa -= 360.0;

  float abs_twa = fabsf(twa);

  // 1. Sprawność żagli w zależności od kąta wiatru (Biegunowa krzywa prędkości)
  float eff = 0.0;
  if (abs_twa < 35.0)
    eff = 0.0; // Kąt martwy (łopoczące żagle)
  else if (abs_twa < 90.0)
    eff = ((abs_twa - 35.0) / 55.0) * 0.50; // Bajdewind -> Półwiatr
  else if (abs_twa <= 135.0)
    eff = 0.50 + (((abs_twa - 90.0) / 45.0) * 0.25); // Baksztag (najwydajniejszy)
  else
    eff = 0.75 - (((abs_twa - 135.0) / 45.0) * 0.15); // Fordewind (nieco wolniej przez zasłanianie żagli)

  // 2. Prędkość z żagli z uwzględnieniem oporu kadłuba
  float max_hull_speed = 9.5;
  float sog_sail = tws * eff;

  // Refowanie żagli i ściana wody: Powyżej prędkości kadłuba wiatr nie daje już prędkości, tylko przechył
  if (sog_sail > max_hull_speed)
  {
    // "Miękkie odcięcie" - pozwala na chwilowy, minimalny zjazd z fali (np. max do 10.5 kn przy sztormie), ale brutalnie hamuje wzrost
    sog_sail = max_hull_speed + (sog_sail - max_hull_speed) * 0.1;
  }

  // 3. Prędkość z silnika (Skalowana od biegu jałowego do max obrotów)
  float sog_engine = 0.0;
  if (eng_on && rpm >= 900.0)
  {
    // 900 RPM = 1.5 kn | 4000 RPM = 8.5 kn
    sog_engine = 1.5 + ((rpm - 900.0) / 3100.0) * 7.0;
  }

  // 4. Hybryda (Motorsailing)
  float final_sog = (sog_sail > sog_engine) ? sog_sail : sog_engine; // Zasadniczo szybszy napęd "wygrywa"

  // Jeśli płyniemy na żaglach i pomagamy sobie silnikiem, dodajemy mały bonus,
  // ale nigdy nie przekraczamy fizycznej prędkości kadłuba.
  if (sog_sail > 2.0 && eng_on && rpm > 1000.0)
  {
    final_sog += 1.0;
  }

  // Twardy limit bezpieczeństwa na wypadek błędów matematycznych
  return constrain(final_sog, 0.0f, 11.0f);
}

void calculateApparentWind(float cog, float sog, float twd, float tws, float &awa, float &aws, float &twa)
{
  // Obliczamy kąt wiatru rzeczywistego względem jachtu (TWA)
  twa = twd - cog;
  while (twa <= -180.0f)
    twa += 360.0f;
  while (twa > 180.0f)
    twa -= 360.0f;

  float twa_rad = twa * PI / 180.0f;
  float w_x = tws * sinf(twa_rad);
  float w_y = tws * cosf(twa_rad) + sog;
  aws = sqrtf(w_x * w_x + w_y * w_y);
  awa = atan2f(w_x, w_y) * 180.0f / PI;
  if (awa < 0)
    awa += 360.0;
}

// --- 3a. FIZYKA JACHTU (Ruch, Wiatr, Silnik) ---
void updateYachtPhysics()
{
  float l_tws, l_twd, l_tgt_ws, l_tgt_wd; //
  float l_depth, l_tgt_depth;
  float l_rpm, l_tgt_rpm;
  float l_awa, l_aws, l_twa;
  float l_cog, l_tgt_nav_cog, l_heading;
  float l_sog, l_tgt_sog;
  float l_lat, l_lon;
  float l_total_log, l_trip_log;
  bool l_eng, l_nav;

  // Optymalizacja: updateYachtPhysics powinna pracować na YachtState przekazanym jako referencja
  // lub bezpośrednio brać mutex raz na całą operację.
  if (!xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
    return;
  l_tws = yacht.tws;
  l_twd = yacht.twd;
  l_tgt_ws = yacht.target_tws;
  l_tgt_wd = yacht.target_twd;
  l_depth = yacht.depth;
  l_tgt_depth = yacht.target_depth;
  l_rpm = yacht.rpm;
  l_tgt_rpm = yacht.target_rpm;
  l_cog = yacht.cog;
  l_tgt_nav_cog = yacht.target_nav_cog;
  l_sog = yacht.sog;
  l_tgt_sog = yacht.target_sog;
  l_lat = yacht.lat;
  l_lon = yacht.lon;
  l_total_log = yacht.total_log;
  l_trip_log = yacht.trip_log;
  l_eng = yacht.engine_on;
  l_nav = yacht.navMode;
  xSemaphoreGive(dataMutex); // Zwalniamy mutex natychmiast po skopiowaniu danych do zmiennych lokalnych

  // Robimy kopię bezpieczeństwa jak by sie fizyka wysypała
  float safe_lat = l_lat;
  float safe_lon = l_lon;
  float safe_sog = l_sog;
  float safe_cog = l_cog;

#ifdef STEADY_WIND_FOR_TESTS
  // Stały wiatr: używamy wartości z platformio.ini (nie modyfikujemy)
  l_tgt_ws = STEADY_WIND_SPEED;
  l_tgt_wd = STEADY_WIND_DIR;
  l_tws = STEADY_WIND_SPEED;
  l_twd = STEADY_WIND_DIR;
#else
  // --- ZOPTYMALIZOWANA POGODA DLA 10 Hz ---
  // random(0, 200) przy 10Hz oznacza zmianę wiatru średnio co 20 sekund (realistycznie)
  if (random(0, 200) == 0)
  {
    // Delikatniejsze skoki wiatru (max +/- 1.5 węzła, a nie 3.0)
    l_tgt_ws += random(-15, 16) / 10.0f;
    l_tgt_ws = constrain(l_tgt_ws, 4.0f, 35.0f); // Od flauty po mocny sztorm

    // Delikatniejsze zmiany kierunku (max +/- 5 stopni, a nie 15)
    l_tgt_wd += random(-5, 6);
    if (l_tgt_wd >= 360.0f)
      l_tgt_wd -= 360.0f;
    if (l_tgt_wd < 0.0f)
      l_tgt_wd += 360.0f;
  }

  // Leniwe wygładzanie prędkości wiatru: 0.005 oznacza, że zmiana o 1 węzeł potrwa kilkanaście sekund
  l_tws += (l_tgt_ws - l_tws) * 0.005f;

  float dir_diff = l_tgt_wd - l_twd;
  if (dir_diff > 180.0f)
    dir_diff -= 360.0f;
  if (dir_diff < -180.0f)
    dir_diff += 360.0f;

  // Zmniejszony próg czułości z 0.5 na 0.1, żeby wiatr mógł leniwie dojść do celu
  if (abs(dir_diff) > 0.1f)
  {
    // Leniwe wygładzanie kierunku: 0.002 to bardzo powolny, realistyczny obrót wiatromierza
    l_twd += dir_diff * 0.002f;
    if (l_twd >= 360.0f)
      l_twd -= 360.0f;
    if (l_twd < 0.0f)
      l_twd += 360.0f;
  }
#endif
  if (random(0, 10) == 0)
    l_tgt_depth = random(20, 800) / 10.0;
  l_depth += (l_tgt_depth - l_depth) * 0.05;

  l_rpm = l_eng ? l_tgt_rpm + random(-15, 16) : 0.0;

  if (l_nav && l_tgt_nav_cog >= 0.0)
  {
    float diff = l_tgt_nav_cog - l_cog;
    if (diff > 180.0)
      diff -= 360.0;
    if (diff < -180.0)
      diff += 360.0;
    if (abs(diff) > 0.5)
    {
      l_cog += diff * 0.2;
      if (l_cog >= 360.0)
        l_cog -= 360.0;
      if (l_cog < 0.0)
        l_cog += 360.0;
    }
  }

  l_tgt_sog = calculateTargetSOG(l_cog, l_twd, l_tws, l_eng, l_rpm);
  l_sog += (l_tgt_sog - l_sog) * 0.05f;

  // Stały krok czasowy 0.1s (100ms) dla płynnego ruchu
  float dist = (l_sog / 3600.0) * 0.1f;
  l_lat += (dist * cosf(l_cog * PI / 180.0)) / 60.0f;
  l_lon += (dist * sinf(l_cog * PI / 180.0)) / (60.0f * cosf(l_lat * PI / 180.0));

  l_total_log += dist;
  l_trip_log += dist;

  // Obliczanie heading (kursu kompasowego) z uwzględnieniem dryfu (leeway).
  // Dryf występuje głównie na żaglach. Gdy silnik pracuje, dryf jest pomijalny.
  float leeway = 0.0f;
  if (!l_eng && l_sog > 1.0f)
  {
    float drift_twa = l_twd - l_cog;
    while (drift_twa <= -180.0f)
      drift_twa += 360.0f;
    while (drift_twa > 180.0f)
      drift_twa -= 360.0f;
    leeway = (drift_twa > 0) ? 4.0f : -4.0f; // Dryf wypycha łódź pod wiatr na mapie, więc HDG musi być "ostrzej"
  }
  l_heading = l_cog + leeway;
  if (l_heading < 0.0f)
    l_heading += 360.0f;
  if (l_heading >= 360.0f)
    l_heading -= 360.0f;

  // Obliczanie wiatru pozornego (AWA/AWS) oraz TWA na podstawie aktualnych parametrów ruchu
  calculateApparentWind(l_cog, l_sog, l_twd, l_tws, l_awa, l_aws, l_twa);

  if (isnan(l_lat) || isnan(l_lon) || isnan(l_sog))
  {
    Serial.println("[BŁĄD FIZYKI] Wykryto NaN! Cofam krok symulacji (Rollback).");

    // Przywracamy stan sprzed ułamka sekundy
    l_lat = safe_lat;
    l_lon = safe_lon;
    l_sog = 0.0;
  }

  // Zapis powrotny
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) // Pobieramy ponownie tylko na czas zapisu
  {
    yacht.tws = l_tws;
    yacht.twd = l_twd;
    yacht.target_tws = l_tgt_ws;
    yacht.target_twd = l_tgt_wd;
    yacht.depth = l_depth;
    yacht.awa = l_awa;
    yacht.aws = l_aws;
    yacht.twa = l_twa;
    yacht.target_depth = l_tgt_depth;
    yacht.rpm = l_rpm;
    yacht.target_rpm = l_tgt_rpm;
    yacht.cog = l_cog;
    yacht.target_nav_cog = l_tgt_nav_cog;
    yacht.sog = l_sog;
    yacht.target_sog = l_tgt_sog;
    yacht.lat = l_lat;
    yacht.lon = l_lon;
    yacht.heading = l_heading;
    yacht.total_log = l_total_log;
    yacht.trip_log = l_trip_log;
    yacht.engine_on = l_eng;
    yacht.navMode = l_nav;
    xSemaphoreGive(dataMutex);
  }
  else
  {
    if (config.log_level >= 1)
      Serial.println("[SYSTEM] Nie udalo sie zapisac fizyki - Mutex zajety");
  }
}
