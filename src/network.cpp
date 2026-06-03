#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "network.h"
#include "config.h"
#include "ais.h"



// BUFORY NMEA DLA KLIENTÓW TCP ---
char tcpBuffers[MAX_TCP_CLIENTS][256];
int tcpIndexes[MAX_TCP_CLIENTS] = {0};

WiFiClient tcpClients[MAX_TCP_CLIENTS];

const int port = 10110;
WiFiUDP udp;
const char *udpAddress = "255.255.255.255";
WiFiServer nmeaServer(port);
WiFiClient tcpClient;

// --- 1. OBSŁUGA KOMEND PRZYCHODZĄCYCH (TCP) ---
void handleIncomingTcpData()
{
  for (int i = 0; i < MAX_TCP_CLIENTS; i++)
  {
    if (tcpClients[i] && tcpClients[i].connected() && tcpClients[i].available())
    {

      while (tcpClients[i].available())
      {
        char c = tcpClients[i].read();

        // 1. Zabezpieczenie przed przepełnieniem
        if (tcpIndexes[i] >= 254)
        {
          tcpIndexes[i] = 0; // Bufor pełny, twardy reset
        }

        // 2. Koniec ramki NMEA
        if (c == '\n' || c == '\r')
        {
          if (tcpIndexes[i] > 0)
          {
            tcpBuffers[i][tcpIndexes[i]] = '\0'; // Zamknięcie tekstu

            // Parsowanie (bez zmian, używamy strstr i strtok)
            if (strstr(tcpBuffers[i], "RMB") != NULL)
            {
              char *token = strtok(tcpBuffers[i], ",");
              int count = 0;
              char *bearingStr = NULL;

              while (token != NULL)
              {
                if (count == 11)
                  bearingStr = token;
                token = strtok(NULL, ",");
                count++;
              }

              if (bearingStr != NULL && strlen(bearingStr) > 0)
              {
                float newCog = atof(bearingStr);
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
                {
                  if (yacht.navMode && !yacht.skipperActive)
                  { // Przyjmuj korekty NMEA tylko gdy tryb Navi aktywny
                    yacht.target_nav_cog = newCog;
                    if (config.log_level >= 2)
                    {
                      Serial.printf(">>> [AUTOPILOT] OpenCPN narzuca korekte: %.1f st.\n", newCog);
                    }
                  }
                  xSemaphoreGive(dataMutex);
                }
                else
                {
                  if (config.log_level >= 1)
                  {
                    Serial.println("[SYSTEM] Ominięto klatkę - Mutex zablokowany!");
                  }
                }
              }
            }
            // Koniec ramki, zerujemy indeks dla tego klienta na kolejne dane
            tcpIndexes[i] = 0;
          }
        }
        // 3. Budowanie ramki znak po znaku
        else
        {
          tcpBuffers[i][tcpIndexes[i]] = c;
          tcpIndexes[i]++;
        }
      }
    }
  }
}
// --- 2. ZARZĄDZANIE POŁĄCZENIAMI TCP ---
void manageNetworkConnections()
{
  if (config.enableTCP)
  {
    // Akceptacja nowych
    if (nmeaServer.hasClient())
    {
      bool clientAdded = false;
      for (int i = 0; i < MAX_TCP_CLIENTS; i++)
      {
        if (!tcpClients[i] || !tcpClients[i].connected())
        {
          if (tcpClients[i])
            tcpClients[i].stop();
          tcpClients[i] = nmeaServer.available();
          tcpClients[i].setNoDelay(true); // Natychmiastowa wysyłka

          // --- KLUCZOWE: CZYSTKA DLA NOWEGO KLIENTA ---
          tcpIndexes[i] = 0;
          memset(tcpBuffers[i], 0, sizeof(tcpBuffers[i]));
          // --------------------------------------------

          if (config.log_level >= 1)
          {
            Serial.printf("[TCP] Nowy klient podłączony do slotu %d z IP: %s\n",
                          i, tcpClients[i].remoteIP().toString().c_str());
          }
          clientAdded = true;
          break;
        }
      }
      /*
       * STARA LOGIKA: Odrzucamy nowych klientów, jeśli wszystkie sloty są zajęte.
       * Nowa logika: Zamiast odrzucać, wyrzucamy najstarszego klienta (slot 0) i przyjmujemy nowego.
       * To zapewnia ciągłość działania nawet przy dużej liczbie prób połączeń.
       * Dodatkowo ustawiamy TCP_NODELAY, aby zminimalizować opóźnienia w komunikacji NMEA.

      if (!clientAdded) {
        WiFiClient rejectClient = nmeaServer.available();
        rejectClient.stop();
        Serial.println("[TCP] Odrzucono klienta - brak wolnych slotów.");
      }
      */
      if (!clientAdded)
      {
        Serial.println("[TCP] Brak slotów. Wyrzucam klienta 0, robię miejsce.");
        tcpClients[0].stop();
        tcpClients[0] = nmeaServer.available();
        tcpClients[0].setNoDelay(true); // Przy okazji wymusza natychmiastowe wysyłanie!
        tcpIndexes[0] = 0;
        memset(tcpBuffers[0], 0, sizeof(tcpBuffers[0]));
      }
    }

    // Czyszczenie martwych
    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
      if (tcpClients[i] && !tcpClients[i].connected())
      {
        if (config.log_level >= 2)
        {
        char ts[24];
        getTimestamp(ts, sizeof(ts));
          Serial.printf("%s [TCP] Wykryto martwe połączenie z adresu %s. Slot nr %d został zwolniony.\n",
                        ts, tcpClients[i].remoteIP().toString().c_str(), i);
        }
        tcpClients[i].stop();
      }
    }
  }
}


void broadcastNmeaToNetwork(const char *pack)
{
  if (config.enableUDP)
  {
    udp.beginPacket(udpAddress, port);
    udp.print(pack);
    udp.endPacket();
  }
  if (config.enableTCP)
  {
    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
      if (tcpClients[i] && tcpClients[i].connected())
      {
        tcpClients[i].print(pack);
      }
    }
  }
}

// --- 4. TRANSMISJA DANYCH STATYCZNYCH AIS ---
void handleStaticAisBroadcasting()
{
  static unsigned long lastStaticAISTick = 0;
  if (millis() - lastStaticAISTick < 60000)
    return;

  lastStaticAISTick = millis();

  bool l_udp_stat = false, l_tcp_stat = false;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)))
  {
    l_udp_stat = config.enableUDP;
    l_tcp_stat = config.enableTCP;
    xSemaphoreGive(dataMutex);
  }
  else
  {
    if (config.log_level >= 1)
    {
      Serial.println("[SYSTEM] Ominięto odczyt statusu transmisji statycznego AIS - Mutex zablokowany!");
    }
    return; // Jeśli nie możemy odczytać statusu transmisji, lepiej nic nie wysyłać
  }

  static char staticPack[700];
  memset(staticPack, 0, sizeof(staticPack)); // ZAWSZE czyść przed użyciem!
  int offset = 0;

  static bool sendPartB = false; // flaga na zmianę A / B
  sendPartB = !sendPartB;        // za każdym razem przełącza

  if (sendPartB)
  {
    // Wysyłamy Part B (callsign + typ + wymiary)
    safeAppend(staticPack, offset, sizeof(staticPack), targets[0].staticNmeaB);
    safeAppend(staticPack, offset, sizeof(staticPack), targets[1].staticNmeaB);
    safeAppend(staticPack, offset, sizeof(staticPack), targets[2].staticNmeaB);

    if (config.log_level >= 2)
    {
      Serial.println(">>> [AIS] Wysłano Type 24B (Part B)");
    }
  }
  else
  {
    // Wysyłamy Part A (tylko nazwa)
    safeAppend(staticPack, offset, sizeof(staticPack), targets[0].staticNmeaA);
    safeAppend(staticPack, offset, sizeof(staticPack), targets[1].staticNmeaA);
    safeAppend(staticPack, offset, sizeof(staticPack), targets[2].staticNmeaA);

    if (config.log_level >= 2)
    {
      Serial.println(">>> [AIS] Wysłano Type 24A (Part A)");
    }
  }

  // Wysyłanie przez UDP / TCP
  if (l_udp_stat)
  {
    udp.beginPacket(udpAddress, port);
    udp.print(staticPack);
    udp.endPacket();
  }

  if (l_tcp_stat)
  {
    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
      if (tcpClients[i] && tcpClients[i].connected())
      {
        tcpClients[i].print(staticPack);
      }
    }
  }
}
