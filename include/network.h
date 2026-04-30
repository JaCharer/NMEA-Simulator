
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"


extern char tcpBuffers[MAX_TCP_CLIENTS][256];
extern int tcpIndexes[MAX_TCP_CLIENTS];

extern WiFiClient tcpClients[MAX_TCP_CLIENTS];

extern const int port;
extern WiFiUDP udp;
extern const char *udpAddress;
extern WiFiServer nmeaServer;
extern WiFiClient tcpClient;

void handleIncomingTcpData();
void manageNetworkConnections();
void broadcastNmeaToNetwork(const char *pack);
void handleStaticAisBroadcasting();