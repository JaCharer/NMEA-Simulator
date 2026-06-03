#ifdef SIMMULATE_NMEA2000
#include "N2K_Handler.h"
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <time.h>
#include <driver/twai.h>
#include <stdlib.h>

// Prywatna funkcja pomocnicza do logowania statusu (Internal Helper)
static void logN2KStatus(bool success, const char* msg) {
    if (config.log_level >= 3) {
        if (success) 
            Serial.printf("[N2K] Wysłano: %s\n", msg);
        else 
            Serial.printf("[BŁĄD N2K] Błąd wysyłki: %s (bufor pełny?)\n", msg);
    }
}

// Handler do diagnostyki zapytań przychodzących
void MsgHandler(const tN2kMsg &N2kMsg) {
    if (N2kMsg.PGN == 59904L && config.log_level >= 2) {
        unsigned long RequestedPGN;
        if (ParseN2kPGN59904(N2kMsg, RequestedPGN)) {
            const char* pgnName = "Nieznany";
            switch (RequestedPGN) {
                case 126996L: pgnName = "Product Information"; break;
                case 126998L: pgnName = "Configuration Information"; break;
                case 60928L:  pgnName = "ISO Address Claim (NAME)"; break;
                case 126464L: pgnName = "PGN Transmit/Receive List"; break;
                default: break;
            }
            Serial.printf("[N2K] Zapytanie o PGN: %lu (%s) od źródła: %d\n", RequestedPGN, pgnName, N2kMsg.Source);
        }
    }
}

// Lista PGN-ów, które to urządzenie będzie wysyłać
// Ważne: PGN 0 oznacza koniec listy
const unsigned long TransmitMessages[] PROGMEM = {
    59904L,  // ISO Request (Nasze zapytania do innych czujników)
    60928L,  // ISO Address Claim (Wysyłane automatycznie przez bibliotekę przy starcie układu)
    126996L, // Product Information (Nasza "wizytówka", wysyłana w odpowiedzi na żądania)
    129025L, // Position, Rapid Update
    129029L, // GNSS Position Data
    129026L, // COG & SOG, Rapid Update
    127250L, // Vessel Heading
    130306L, // Wind Data
    128267L, // Water Depth
    128259L, // Speed
    128275L, // Distance Log
    127488L, // Engine Parameters, Rapid Update
    0        // Koniec listy
};

const unsigned long ReceiveMessages[] PROGMEM = {
    // ==========================================
    // 0. SYSTEMOWE (Zarządzanie siecią)
    // ==========================================
    60928L,  // ISO Address Claim (Do budowania listy urządzeń / znalezienia czujników)
    126996L,  // Product Information (Czytelne nazwy modeli i wersje softu)
    0
};

void setupN2K() {
    // Zwiększamy znacząco bufor, aby obsłużyć 4 urządzenia odpowiadające na raz
    NMEA2000.SetN2kCANMsgBufSize(20); 
    NMEA2000.SetN2kCANSendFrameBufSize(200);

    NMEA2000.SetDeviceCount(4);

    // Konfiguracja KAŻDEGO urządzenia z osobnym indeksem (ostatni parametr)
    // Parametry: Serial, ProductCode, ModelID, SwCode, ModelVer, LoadEquiv, N2kVer, CertLevel, Index
    // Urządzenie 0: GPS/Nav
    NMEA2000.SetProductInformation("SIM-GPS-01", 100, "Virtual GPS", "1.2.0", "1.0", 1, 1300, 1, 0);
    NMEA2000.SetDeviceInformation(100001, 145, 60, 145, 4, 0);

    // Urządzenie 1: Wind
    NMEA2000.SetProductInformation("SIM-WIND-01", 101, "Virtual Wind", "1.2.0", "1.0", 1, 1300, 1, 1);
    NMEA2000.SetDeviceInformation(100002, 130, 85, 130, 4, 1);

    // Urządzenie 2: Depth/Speed
    NMEA2000.SetProductInformation("SIM-DEPTH-01", 102, "Virtual Depth", "1.2.0", "1.0", 1, 1300, 1, 2);
    NMEA2000.SetDeviceInformation(100003, 150, 60, 150, 4, 2);

    // Urządzenie 3: Engine
    NMEA2000.SetProductInformation("SIM-ENG-01", 103, "Virtual Engine", "1.2.0", "1.0", 1, 1300, 1, 3);
    NMEA2000.SetDeviceInformation(100004, 140, 50, 140, 4, 3);

    // Rejestracja listy PGN dla każdego urządzenia z osobna (Kluczowe dla PGN 126464)
    for(int i=0; i<4; i++) {
        NMEA2000.ExtendTransmitMessages(TransmitMessages, i);
        NMEA2000.ExtendReceiveMessages(ReceiveMessages, i);
    }

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22); // Urządzenie 0 dostaje adres 22
    NMEA2000.SetMsgHandler(MsgHandler); // Rejestracja handlera diagnostycznego
    bool ok = NMEA2000.Open();

    if (config.log_level >= 1) {
        if (ok) {
            Serial.println("[N2K] Stos NMEA 2000 podniesiony. Symulacja 4 urządzeń aktywna.");
        } else {
            Serial.println("[BŁĄD N2K] Nie udało się otworzyć magistrali CAN!");
        }
    }
}

void sendN2KNavigation(const YachtState &snap) {
    // Nie wysyłaj, dopóki urządzenie nie wywalczy adresu w sieci
    if (NMEA2000.GetN2kSource(0) == 255) return;

    tN2kMsg N2kMsg;
    bool success = true;

    // Position Rapid (Podstawowe współrzędne)
    SetN2kPGN129025(N2kMsg, snap.lat, snap.lon); 
    success &= NMEA2000.SendMsg(N2kMsg, 0);

    // GNSS Position Data (Szczegółowe dane GPS - PGN 129029)
    time_t now = time(NULL);
    uint16_t days = now / 86400;
    double seconds = now % 86400;

    // Parametry: SID, Dni, Sekundy, Lat, Lon, Alt, Typ GNSS, Metoda, Satelity, HDOP, PDOP
    SetN2kGNSS(N2kMsg, 1, days, seconds, snap.lat, snap.lon, 0.0, N2kGNSSt_GPS, N2kGNSSm_GNSSfix, 12, 0.8, 1.0);
    success &= NMEA2000.SendMsg(N2kMsg, 0);

    // COG/SOG Rapid
    SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, DegToRad(snap.cog), KnotsToms(snap.sog));
    success &= NMEA2000.SendMsg(N2kMsg, 0);

    // Heading
    SetN2kTrueHeading(N2kMsg, 1, DegToRad(snap.heading));
    success &= NMEA2000.SendMsg(N2kMsg, 0);
    logN2KStatus(success, "Nav (PGN 129025, 129029, 129026, 127250)");
}

void sendN2KWind(const YachtState &snap) {
    if (NMEA2000.GetN2kSource(1) == 255) return;

    tN2kMsg N2kMsg;
    bool success = true;

    // Wind Apparent
    SetN2kWindSpeed(N2kMsg, 1, KnotsToms(snap.aws), DegToRad(snap.awa), N2kWind_Apparent);
    success &= NMEA2000.SendMsg(N2kMsg, 1);

    // Wind True
    SetN2kWindSpeed(N2kMsg, 1, KnotsToms(snap.tws), DegToRad(snap.twa), N2kWind_True_boat);
    success &= NMEA2000.SendMsg(N2kMsg, 1);
    logN2KStatus(success, "Wind (PGN 130306 App/True)");
}

void sendN2KDepth(const YachtState &snap) {
    if (NMEA2000.GetN2kSource(2) == 255) return;

    tN2kMsg N2kMsg;
    bool success = true;

    SetN2kWaterDepth(N2kMsg, 1, snap.depth, 1.5); // Offset 1.5m
    success &= NMEA2000.SendMsg(N2kMsg, 2);

    // Dodano: Prędkość przez wodę (STW)
    // snap.sog w symulatorze służy jako STW jeśli nie ma prądu
    SetN2kBoatSpeed(N2kMsg, 1, KnotsToms(snap.sog), N2kDoubleNA, N2kSWRT_Paddle_wheel);
    success &= NMEA2000.SendMsg(N2kMsg, 2);

    // Pobranie czasu systemowego do logu
    time_t now = time(NULL);
    uint16_t days = now / 86400;
    double seconds = now % 86400;

    SetN2kDistanceLog(N2kMsg, days, seconds, (uint32_t)(snap.total_log * 1852), (uint32_t)(snap.trip_log * 1852)); // NM -> meters
    success &= NMEA2000.SendMsg(N2kMsg, 2);
    logN2KStatus(success, "Depth/Speed/Log (PGN 128267, 128259, 128275)");
}

void sendN2KEngine(const YachtState &snap) {
    if (NMEA2000.GetN2kSource(3) == 255) return;

    if (snap.engine_on) {
        tN2kMsg N2kMsg;
        SetN2kEngineParamRapid(N2kMsg, 0, snap.rpm);
        logN2KStatus(NMEA2000.SendMsg(N2kMsg, 3), "Engine RPM (PGN 127488)");
    }
}

void runN2KUpdate() {
    NMEA2000.ParseMessages();

    static unsigned long lastHealthCheck = 0;

    if (config.log_level >= 1 && (millis() - lastHealthCheck > 5000)) {
        lastHealthCheck = millis();

        // 1. Diagnostyka warstwy fizycznej (ESP32 TWAI)
        twai_status_info_t status;
        esp_err_t err = twai_get_status_info(&status);
        if (err == ESP_OK) {
            if (status.state == TWAI_STATE_BUS_OFF) {
                Serial.println("[ALARM N2K] BUS-OFF! Magistrala zablokowana (zwarcie lub brak rezystorów).");
            }
            else if (status.tx_error_counter > 127) {
                // To jest stan, w którym ESP32 wejdzie, gdy nie ma transcievera (brak ACK)
                Serial.printf("[ALARM N2K] Error Passive! TEC: %d. Brak transcievera lub sieci?\n", status.tx_error_counter);
            }
            else if (status.tx_error_counter > 0) {
                if (config.log_level >= 3) {
                    Serial.printf("[N2K WARN] Wykryto błędy na linii. TEC: %d\n", status.tx_error_counter);
                }
            } else if (status.tx_error_counter == 0 && status.state == TWAI_STATE_RUNNING) {
                if (config.log_level >= 3) {
                    Serial.println("[N2K Info] Magistrala CAN aktywna, wykryto ACK.");
                }
            }
        } else {
            // Jeśli nie jest ESP_OK, wyświetlamy kod błędu
            if (config.log_level >= 3) {
                Serial.printf("[BŁĄD TWAI] Nie można pobrać statusu! Kod: 0x%X\n", err);
            }
        }
        
        // 2. Diagnostyka warstwy logicznej (NMEA 2000 Address Claiming)
        uint8_t addr = NMEA2000.GetN2kSource(0);
        if (addr == 255) {
            Serial.println("[ALARM N2K] Urządzenie nie uzyskało adresu (255)!");
        } else if (config.log_level >= 3) {
            Serial.printf("[N2K Health] Adres źródłowy urządzenia 0: %d\n", addr);
        }
    }
}

#endif

#