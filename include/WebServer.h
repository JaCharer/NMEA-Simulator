#pragma once
#include <ESPAsyncWebServer.h>
#include "config.h"

// Udostępniamy obiekty serwera i socketów dla innych plików (.cpp)
extern AsyncWebServer webServer;
extern AsyncWebSocket ws;
extern AsyncWebSocket skWs;


void setupWebServer();

void updateBrowserDashboard(const YachtState &snap);