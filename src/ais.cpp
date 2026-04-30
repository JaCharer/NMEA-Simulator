#include <Arduino.h>
#include "ais.h"
#include "arpa.h"
#include "NMEA0183.h"
#include "config.h"

// Definicja tablicy targets (przeniesiona z nagłówka, by uniknąć błędów multiple definition)
Target targets[3] = {
    {1, 55.5500, 18.0500, 210.0, 16.0, "PROM", 15.0, 16.0, 24.0, false, 210.0, 210.0, false, 0, 1.5, "", ""},
    {2, 55.4500, 17.9500, 45.0, 10.0, "TANKOWIEC", 20.0, 10.0, 15.0, false, 45.0, 45.0, false, 0, 0.2, "", ""},
    {3, 55.5000, 18.1000, 0.0, 0.0, "RYBAK", 6.0, 0.0, 12.0, false, 0.0, 0.0, false, 0, 4.0, "", ""}
};


class FastAISBitWriter {
private:
    uint8_t payload[40];
    int bitIndex;
public:
    FastAISBitWriter() {
        memset(payload, 0, sizeof(payload));
        bitIndex = 0;
    }
    void pushBits(uint32_t value, int numBits) {
        for (int i = numBits - 1; i >= 0; i--) {
            if ((value >> i) & 1)
                payload[bitIndex / 8] |= (1 << (7 - (bitIndex % 8)));
            bitIndex++;
        }
    }
    void pushString(const char *str, int maxChars) {
        for (int i = 0; i < maxChars; i++) {
            char c = (i < (int)strlen(str)) ? str[i] : '@';
            uint32_t val = 0;
            if (c >= '@' && c <= '_') val = c - 0x40;
            else if (c >= ' ' && c <= '?') val = c - 0x20 + 32;
            pushBits(val, 6);
        }
    }
    void encodeToAscii(char *outputAscii, int &padding) {
        int numChars = (bitIndex + 5) / 6;
        padding = (numChars * 6) - bitIndex;
        for (int i = 0; i < numChars; i++) {
            uint8_t sixBits = 0;
            for (int b = 0; b < 6; b++) {
                int absBit = (i * 6) + b;
                if (absBit < bitIndex && (payload[absBit / 8] & (1 << (7 - (absBit % 8))))) {
                    sixBits |= (1 << (5 - b));
                }
            }
            sixBits += 0x30;
            if (sixBits > 0x57) sixBits += 0x08;
            outputAscii[i] = sixBits;
        }
        outputAscii[numChars] = '\0';
    }
};

void generateAIVDM(int mmsi, float lat, float lon, float sog, float cog, float heading, char *outputLine) {
    FastAISBitWriter aisWriter;
    int32_t i_lon = (int32_t)(lon * 600000.0);
    int32_t i_lat = (int32_t)(lat * 600000.0);
    uint32_t i_sog = (uint32_t)(sog * 10.0);
    if (i_sog > 1022) i_sog = 1022;
    uint32_t i_cog = (uint32_t)(cog * 10.0);
    if (i_cog > 3599) i_cog = 3599;
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
    aisWriter.pushBits(i_hdg, 9);
    aisWriter.pushBits(60, 6);
    aisWriter.pushBits(0, 2);
    aisWriter.pushBits(0, 3);
    aisWriter.pushBits(0, 1);
    aisWriter.pushBits(0, 19);

    char aisPayload[45];
    int padding;
    aisWriter.encodeToAscii(aisPayload, padding);
    char baseLine[120];
    sprintf(baseLine, "!AIVDM,1,1,,A,%s,%d", aisPayload, padding);
    addNMEAChecksumToLine(baseLine, outputLine);
}

void generateType24A(int mmsi, const char *name, char *outputLine) {
    FastAISBitWriter aisWriter;
    aisWriter.pushBits(24, 6);
    aisWriter.pushBits(0, 2);
    aisWriter.pushBits(mmsi, 30);
    aisWriter.pushBits(0, 2);
    aisWriter.pushString(name, 20);
    char aisPayload[40];
    int padding;
    aisWriter.encodeToAscii(aisPayload, padding);
    char baseLine[100];
    sprintf(baseLine, "!AIVDM,1,1,,A,%s,%d", aisPayload, padding);
    addNMEAChecksumToLine(baseLine, outputLine);
}

void generateType24B(int mmsi, const char *callsign, uint8_t shipType, float length_m, float beam_m, char *outputLine) {
    FastAISBitWriter aisWriter;
    aisWriter.pushBits(24, 6);
    aisWriter.pushBits(0, 2);
    aisWriter.pushBits(mmsi, 30);
    aisWriter.pushBits(1, 2);
    aisWriter.pushBits(shipType, 8);
    aisWriter.pushBits(0, 42);
    aisWriter.pushString(callsign, 7);

    uint32_t dimA = (uint32_t)(length_m * 0.1f);
    uint32_t dimB = 0;
    uint32_t dimC = (uint32_t)(beam_m * 0.1f / 2);
    uint32_t dimD = dimC;

    aisWriter.pushBits(dimA, 9);
    aisWriter.pushBits(dimB, 9);
    aisWriter.pushBits(dimC, 6);
    aisWriter.pushBits(dimD, 6);
    aisWriter.pushBits(0, 6);

    char aisPayload[45];
    int padding;
    aisWriter.encodeToAscii(aisPayload, padding);
    char baseLine[120];
    sprintf(baseLine, "!AIVDM,1,1,,A,%s,%d", aisPayload, padding);
    addNMEAChecksumToLine(baseLine, outputLine);
}

// --- 3c. LOGIKA DYNAMICZNA AIS (Inne Statki + Kolizje) ---
// --- 3c-1. LOGIKA DECYZYJNA POJEDYNCZEGO CELU AIS (Unikanie kolizji + Fizyka) ---
void processAisTargetBehavior(Target &target, float yachtLat, float yachtLon,
                              float yachtSog, float yachtCog, bool yachtEng,
                              int interval)
{
  unsigned long nowMs = millis();

  // --- Obliczenia geometryczne ---
  float dx = (yachtLon - target.lon) * cosf(yachtLat * PI / 180.0);
  float dy = yachtLat - target.lat;
  float dist_nm = sqrtf(dx * dx + dy * dy) * 60.0;

  float bearing_to_jacht = atan2(dx, dy) * 180.0 / PI;
  if (bearing_to_jacht < 0)
    bearing_to_jacht += 360.0;

  bool borderRisk = (dist_nm > target.max_radius_nm);
  unsigned long timeSinceLastDecision = nowMs - target.lastCourseDecision;

  // --- RYBAK (ID 3) ---
  if (target.id == 3)
  {
    if (borderRisk)
      target.is_returning = true;
    else if (dist_nm <= 1.0)
      target.is_returning = false;

    if (target.is_returning)
    {
      target.sog = target.max_sog;
      if (timeSinceLastDecision > 5000)
      {
        target.target_cog = bearing_to_jacht;
        target.lastCourseDecision = nowMs;
        if (config.log_level >= 2) {
          char ts[24];
          getTimestamp(ts, sizeof(ts));
          Serial.printf("%s [AIS DEBUG] %s wraca do strefy.\n", ts, target.name);
        }
      }
    }
    else
    {
      target.sog = 0.0;
    }
  }
  else
  {
    // --- PROM / TANKOWIEC ---
    if (borderRisk)
    {
      target.sog = target.max_sog;
      if (timeSinceLastDecision > 15000)
      {
        target.target_cog = bearing_to_jacht + random(-20, 21);
        target.planned_cog = target.target_cog;
        target.is_avoiding = false;
        target.lastCourseDecision = nowMs;
      }
    }
    else
    {
      bool arpa_alarm = checkCollisionRisk_ARPA(yachtLat, yachtLon, yachtSog, yachtCog,
                                                target.lat, target.lon, target.sog, target.cog);

      bool i_am_give_way = false;
      if (arpa_alarm)
      {
        float rel = bearing_to_jacht - target.cog;
        while (rel > 180)
          rel -= 360;
        while (rel < -180)
          rel += 360;

        if (fabsf(rel) < 15.0f)
          i_am_give_way = true;
        else if (fabsf(rel) < 112.5f)
          i_am_give_way = (!yachtEng) || (rel > 0);
        else
          i_am_give_way = true;
      }

      if (arpa_alarm && i_am_give_way && !target.is_avoiding)
      {
        target.planned_cog = target.target_cog;
        target.is_avoiding = true;
        if (config.log_level >= 2)
        {
          char ts[24];
          getTimestamp(ts, sizeof(ts));
          Serial.printf("%s [AIS COLREG] %s – START unikania\n", ts, target.name);
        }
      }

      if (target.is_avoiding)
      {
        float parallel_cog = yachtCog + 180.0f;
        if (parallel_cog >= 360.0f)
          parallel_cog -= 360.0f;
        target.target_cog = parallel_cog;

        if (target.sog < target.base_sog)
        {
          target.sog += 1.2f;
          if (target.sog > target.base_sog)
            target.sog = target.base_sog;
        }

        float rel = bearing_to_jacht - target.cog;
        while (rel > 180)
          rel -= 360;
        while (rel < -180)
          rel += 360;

        bool pastAndClear = (fabsf(rel) > 110.0f && dist_nm > 1.3f) || (dist_nm > 4.0f);

        if (pastAndClear)
        {
          target.target_cog = target.planned_cog;
          target.is_avoiding = false;
          if (config.log_level >= 2)
          {
            char ts[24];
            getTimestamp(ts, sizeof(ts));
            Serial.printf("%s [AIS COLREG] %s – Past & Clear → wraca na kurs %.0f°\n",
                          ts, target.name, target.planned_cog);
          }
        }
      }
      else
      {
        if (target.sog < target.base_sog)
          target.sog = target.base_sog;

        if (timeSinceLastDecision > 90000)
        {
          target.target_cog = target.planned_cog + random(-22, 23);
          target.planned_cog = target.target_cog;
          target.lastCourseDecision = nowMs;
        }
      }
    }
  }

  // --- RUCH CELOW (zawsze na końcu) ---
  // 1. TWORZYMY KOPIĘ BEZPIECZEŃSTWA (przed obliczeniami)
  float safe_lat = target.lat;
  float safe_lon = target.lon;
  float safe_sog = target.sog;
  float safe_cog = target.cog;

  // 2. FIZYKA SKRĘTU I RUCHU
  float diff = target.target_cog - target.cog;
  if (diff > 180.0)
    diff -= 360.0;
  if (diff < -180.0)
    diff += 360.0;

  float turn = constrain(diff, -target.max_turn_rate, target.max_turn_rate) * interval;
  target.cog += turn;

  float d = (target.sog / 3600.0) * interval;
  target.lat += (d * cosf(target.cog * PI / 180.0)) / 60.0f;
  target.lon += (d * sinf(target.cog * PI / 180.0)) / (60.0f * cosf(target.lat * PI / 180.0));

  // 3. INTELIGENTNY OCHRONIARZ (Rollback w razie NaN)
  if (isnan(target.lat) || isnan(target.lon))
  {
    Serial.printf("[BŁĄD FIZYKI AIS] Wykryto NaN dla celu: %s! Cofam krok symulacji.\n", target.name);

    // Przywracamy statek na miejsce sprzed ułamka sekundy
    target.lat = safe_lat;
    target.lon = safe_lon;

    // Zatrzymujemy statek, aby nie wpadł w pętlę błędów z dużą prędkością
    target.sog = 0.0;

    // Odbijamy kurs o 1 stopień, by uciec z "martwego punktu matematycznego"
    target.cog = safe_cog + 1.0f;
    if (target.cog >= 360.0f)
      target.cog -= 360.0f;
  }
}

// --- 3c-2. GŁÓWNA PĘTLA AIS ---
void updateAisTargetsAndSentences(char *pack, int &offset, const YachtState &snap)
{

  float step_interval = 1.0f;

  for (int i = 0; i < 3; i++)
  {
    // Wywołanie wydzielonej logiki
    processAisTargetBehavior(targets[i], snap.lat, snap.lon, snap.sog, snap.cog, snap.engine_on, step_interval);

    // Generowanie ramki NMEA po aktualizacji
    char finalLine[160];
    int mmsi = 261000000 + targets[i].id;
    float current_hdg = targets[i].cog;

    if (targets[i].id == 3 && targets[i].sog < 0.5)
      current_hdg = 511.0;

    generateAIVDM(mmsi, targets[i].lat, targets[i].lon, targets[i].sog, targets[i].cog, current_hdg, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);
  }
}
