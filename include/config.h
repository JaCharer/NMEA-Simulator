#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>

// Definicja konfiguracji systemu
struct SystemConfig {
  String wifi_ssid = "";
  String wifi_pass = "";
  bool useAP_mode = false;
  bool enableUDP = true;
  bool enableTCP = true;
  bool enableSignalK = false;
  int log_level = 1; // 0:NONE, 1:BASIC, 2:ADVANCED, 3:EXPERT, 4:ULTRA
};

// Definicja stanu jachtu
struct YachtState
{
  float lat = 55.5000, lon = 18.0000;
  float cog = 0.0, sog = 0.0, target_sog = 0.0;
  float heading = 0.0; 
  float depth = 30.0, target_depth = 30.0;
  float twd = 0.0, tws = 0.0;               
  float target_twd = 0.0, target_tws = 0.0; 
  float awa = 0.0, aws = 0.0, twa = 0.0;    
  float rpm = 0.0, target_rpm = 900.0;
  bool engine_on = false;
  bool navMode = false;        
  float target_nav_cog = -1.0; 
  float total_log = 0.0;       
  float trip_log = 0.0;        
  bool skipperActive = false;  
  int currentWaypoint = 0;     
};

// --- KONFIGURACJA BUFORÓW ---
#define NMEA_BUFFER_SIZE 2048 // Maksymalny rozmiar jednego pakietu danych (NMEA + AIS)
#define MAX_TCP_CLIENTS 3



void getTimestamp(char* dest, size_t size);
void safeAppend(char *buffer, int &offset, int maxSize, const char *line);

// Deklaracje zmiennych globalnych (implementacja znajduje się w main.cpp)
extern YachtState yacht;
extern SystemConfig config;
extern SemaphoreHandle_t dataMutex;
extern Preferences preferences;
