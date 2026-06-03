#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <sys/time.h>
#include <time.h>
#include "config.h"


// --- STRUKTURA FLOTY (Cele AIS) ---
struct Target
{
  int id;
  float lat, lon, cog, sog; // Podstawowe dane pozycyjne i ruchu
  const char *name;         // Nazwa statku (dla AIS)

  float max_radius_nm; // Promień strefy (smycz)
  float base_sog;      // Nominalna prędkość statku
  float max_sog;       // Prędkość pościgowa (efekt gumki)
  bool is_returning;   // Flaga dla Rybaka

  float target_cog;  // Kurs, na który aktualnie skręca statek
  float planned_cog; // Kurs, do którego statek wróci po manewrze
  bool is_avoiding;  // Flaga: Czy statek jest w trakcie unikania kolizji

  unsigned long lastCourseDecision;
  float max_turn_rate; // Bezwładność: max. stopnie skrętu na sekundę

  char staticNmeaA[120]; // Cache dla ramki AIS Typ 24A
  char staticNmeaB[120]; // Cache dla ramki AIS Typ 24B
};


void generateAIVDM(int mmsi, float lat, float lon, float sog, float cog, float heading,
                   char *outputLine);

void generateType24A(int mmsi, const char *name, char *outputLine);

void generateType24B(int mmsi, const char *callsign, uint8_t shipType, float length_m,
                     float beam_m, char *outputLine);

void processAisTargetBehavior(Target &target, float yachtLat, float yachtLon,
                              float yachtSog, float yachtCog, bool yachtEng,
                              int interval);

void updateAisTargetsAndSentences(char *pack, int &offset, const YachtState &snap);

extern Target targets[3];
