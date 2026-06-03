#include <Arduino.h>

// =========================================================================
// ZMIENNE GLOBALNE I STRUKTURY
// =========================================================================

// Struktura bufora dla wiadomości wieloczęściowych (zabezpiecza przed przeplataniem)
#define MAX_BUFFERS 4
struct AisMultipart
{
    int msgId;
    char channel; // 'A' lub 'B'
    String payload;
    unsigned long timestamp; // Do usuwania starych, niepełnych wiadomości
};
AisMultipart mpBuffers[MAX_BUFFERS];

// =========================================================================
// FUNKCJE POMOCNICZE (OPERACJE BITOWE ZAMIAST STRINGÓW - OSZCZĘDNOŚĆ RAMU!)
// =========================================================================

// Dekoduje pojedynczy znak AIS na 6-bitową wartość
uint8_t decode6bit(char c)
{
    uint8_t val = c - 0x30;
    if (val > 40)
        val -= 8;
    return val & 0x3F;
}

// Magiczna funkcja: Wyciąga dowolną ilość bitów (max 32) prosto z tablicy znaków!
uint32_t getBits(const char *payload, int startBit, int numBits)
{
    uint32_t result = 0;
    for (int i = 0; i < numBits; i++)
    {
        int bitPos = startBit + i;
        int charIdx = bitPos / 6;
        int bitInChar = 5 - (bitPos % 6); // MSB to bit 5

        if (payload[charIdx] == 0)
            break; // Koniec łańcucha

        uint8_t cVal = decode6bit(payload[charIdx]);
        uint8_t bitVal = (cVal >> bitInChar) & 0x01;
        result = (result << 1) | bitVal;
    }
    return result;
}

// To samo co wyżej, ale dla wartości ujemnych (np. współrzędne geograficzne)
long getSignedBits(const char *payload, int startBit, int numBits)
{
    uint32_t val = getBits(payload, startBit, numBits);
    if (val & (1UL << (numBits - 1)))
    { // Sprawdzamy bit znaku
        uint32_t mask = ~0UL << numBits;
        return (long)(val | mask);
    }
    return (long)val;
}

// Wyciąga zdekodowany tekst ASCII prosto z bitów
String getAisString(const char *payload, int startBit, int numChars)
{
    String result = "";
    for (int i = 0; i < numChars; i++)
    {
        uint8_t val = getBits(payload, startBit + i * 6, 6);
        char c = (val < 32) ? (val + 64) : val;
        if (c != '@')
            result += c; // Pomijamy padding AIS
    }
    result.trim(); // Usuwamy spacje na końcu
    return result;
}

// Walidacja sumy kontrolnej NMEA
bool validateChecksum(const String &nmeaLine)
{
    int asterisk = nmeaLine.lastIndexOf('*');
    if (asterisk < 1 || asterisk + 2 >= nmeaLine.length())
        return false;

    uint8_t calculated = 0;
    for (int i = 1; i < asterisk; i++)
    {
        calculated ^= nmeaLine[i];
    }

    const char *checksumPart = &nmeaLine.c_str()[asterisk + 1];
    return calculated == (uint8_t)strtol(checksumPart, nullptr, 16);
}

// Deklaracje funkcji dekodujących
void decodeAIS_Type1(const char *payload);
void decodeAIS_Type5(const char *payload);
void decodeAIS_Type18(const char *payload);
void decodeAIS_Type24(const char *payload);

// =========================================================================
// GŁÓWNY WRAPPER ODBIORCZY NMEA
// =========================================================================
void processNMEALine(const String &nmeaLine)
{
    if (!nmeaLine.startsWith("!AIVDM") && !nmeaLine.startsWith("!AIVDO"))
        return;

    // --- KROK 1: Bezpieczeństwo (Checksum) ---
    if (!validateChecksum(nmeaLine))
    {
        Serial.println("[BŁĄD] Odrzucono ramkę - zła suma kontrolna.");
        return;
    }

    // --- KROK 2: Parsowanie nagłówka NMEA (użycie sscanf zamiast wielu substringów) ---
    // Przykład: !AIVDM,2,1,3,B,p000h`v01p@S<8P,2*35
    char lineBuf[128];
    strncpy(lineBuf, nmeaLine.c_str(), sizeof(lineBuf));
    lineBuf[sizeof(lineBuf)-1] = '\0';

    char *tokens[7];
    int tCount = 0;
    char *p = lineBuf;
    
    // Ręczne tokenizowanie po przecinku, by uniknąć problemów z pustymi polami (,,)
    while (p && tCount < 7) {
        tokens[tCount++] = p;
        p = strchr(p, ',');
        if (p) {
            *p = '\0';
            p++;
        }
    }

    if (tCount < 6) return;

    int totalParts = atoi(tokens[1]);
    int partNum = atoi(tokens[2]);
    int msgId = (strlen(tokens[3]) > 0) ? atoi(tokens[3]) : 0;
    char channel = tokens[4][0]; 
    const char* payload = tokens[5];

    // --- KROK 3: Wybór obsługi ---
    if (totalParts == 1)
    {
        // [WIADOMOŚCI JEDNOCZĘŚCIOWE]
        uint8_t msgType = getBits(payload, 0, 6);

        if (msgType >= 1 && msgType <= 3)
            decodeAIS_Type1(payload);
        else if (msgType == 5)
            decodeAIS_Type5(payload);
        else if (msgType == 18)
            decodeAIS_Type18(payload);
        else if (msgType == 24)
            decodeAIS_Type24(payload);
    }
    else if (totalParts == 2)
    {
        // [WIADOMOŚCI WIELOCZĘŚCIOWE - ZARZĄDZANIE BUFOREM]
        unsigned long now = millis();

        if (partNum == 1)
        {
            // Szukamy wolnego slotu lub nadpisujemy stary (starszy niż 5 sekund)
            int slot = -1;
            for (int i = 0; i < MAX_BUFFERS; i++)
            {
                if (mpBuffers[i].payload.length() == 0 || (now - mpBuffers[i].timestamp > 5000))
                {
                    slot = i;
                    break;
                }
            }
            if (slot != -1)
            {
                mpBuffers[slot].msgId = msgId;
                mpBuffers[slot].channel = channel;
                mpBuffers[slot].payload = String(payload);
                mpBuffers[slot].timestamp = now;
            }
        }
        else if (partNum == 2)
        {
            // Szukamy pasującej pierwszej części
            for (int i = 0; i < MAX_BUFFERS; i++)
            {
                if (mpBuffers[i].msgId == msgId && mpBuffers[i].channel == channel && mpBuffers[i].payload.length() > 0)
                {

                    String fullPayload = mpBuffers[i].payload + String(payload);
                    uint8_t msgType = getBits(fullPayload.c_str(), 0, 6);

                    if (msgType == 5)
                        decodeAIS_Type5(fullPayload.c_str());
                    // Tutaj można dodać inne długie wiadomości

                    mpBuffers[i].payload = ""; // Czyścimy slot
                    break;
                }
            }
        }
    }
}

// =========================================================================
// DEKODER: Typ 1, 2, 3 (Pozycja - Klasa A)
// =========================================================================
void decodeAIS_Type1(const char *payload)
{
    long mmsi = getBits(payload, 8, 30);
    int navStatus = getBits(payload, 38, 4);
    int rot = getSignedBits(payload, 42, 8);
    float sog = getBits(payload, 50, 10) / 10.0f;
    float lon = getSignedBits(payload, 61, 28) / 600000.0f;
    float lat = getSignedBits(payload, 89, 27) / 600000.0f;
    float cog = getBits(payload, 116, 12) / 10.0f;
    int heading = getBits(payload, 128, 9);

    Serial.printf("\n[AIS] Klasa A Pozycja (Typ 1/2/3)\n");
    Serial.printf(" MMSI: %09ld | SOG: %.1f w | COG: %.1f st\n", mmsi, sog, cog);
    Serial.printf(" Lat: %.6f | Lon: %.6f | HDG: %d\n", lat, lon, heading);
}

// =========================================================================
// DEKODER: Typ 18 (Pozycja - Klasa B - Jachty, Motorówki)
// =========================================================================
void decodeAIS_Type18(const char *payload)
{
    long mmsi = getBits(payload, 8, 30);
    float sog = getBits(payload, 46, 10) / 10.0f;
    float lon = getSignedBits(payload, 57, 28) / 600000.0f;
    float lat = getSignedBits(payload, 85, 27) / 600000.0f;
    float cog = getBits(payload, 112, 12) / 10.0f;
    int heading = getBits(payload, 124, 9);

    Serial.printf("\n[AIS] Klasa B Pozycja (Typ 18)\n");
    Serial.printf(" MMSI: %09ld | SOG: %.1f w | COG: %.1f st\n", mmsi, sog, cog);
    Serial.printf(" Lat: %.6f | Lon: %.6f | HDG: %d\n", lat, lon, heading);
}

// =========================================================================
// DEKODER: Typ 5 (Dane statyczne - Klasa A)
// =========================================================================
void decodeAIS_Type5(const char *payload)
{
    long mmsi = getBits(payload, 8, 30);
    long imo = getBits(payload, 40, 30);
    String callsign = getAisString(payload, 70, 7);
    String shipName = getAisString(payload, 112, 20);
    int shipType = getBits(payload, 232, 8);
    int dimA = getBits(payload, 240, 9);
    int dimB = getBits(payload, 249, 9);
    int dimC = getBits(payload, 258, 6);
    int dimD = getBits(payload, 264, 6);
    float draught = getBits(payload, 270, 8) / 10.0f;

    Serial.printf("\n[AIS] Klasa A Dane Statyczne (Typ 5)\n");
    Serial.printf(" MMSI: %09ld | Nazwa: %s\n", mmsi, shipName.c_str());
    Serial.printf(" Callsign: %s | IMO: %ld | Typ: %d\n", callsign.c_str(), imo, shipType);
    Serial.printf(" Wymiary: %dx%d m | Zanurzenie: %.1f m\n", (dimA + dimB), (dimC + dimD), draught);
}

// =========================================================================
// DEKODER: Typ 24 (Dane statyczne - Klasa A i B)
// =========================================================================
void decodeAIS_Type24(const char *payload)
{
    long mmsi = getBits(payload, 8, 30);
    int partNumber = getBits(payload, 38, 2);

    Serial.printf("\n[AIS] Dane Statyczne (Typ 24 - Część %c)\n", partNumber == 0 ? 'A' : 'B');
    Serial.printf(" MMSI: %09ld\n", mmsi);

    if (partNumber == 0)
    {
        String shipName = getAisString(payload, 40, 20);
        Serial.printf(" Nazwa: %s\n", shipName.c_str());
    }
    else if (partNumber == 1)
    {
        int shipType = getBits(payload, 40, 8);
        String callsign = getAisString(payload, 90, 7);
        int dimA = getBits(payload, 132, 9);
        int dimB = getBits(payload, 141, 9);
        int dimC = getBits(payload, 150, 6);
        int dimD = getBits(payload, 156, 6);

        Serial.printf(" Callsign: %s | Typ: %d\n", callsign.c_str(), shipType);
        Serial.printf(" Wymiary: %dx%d m\n", (dimA + dimB), (dimC + dimD));
    }
}