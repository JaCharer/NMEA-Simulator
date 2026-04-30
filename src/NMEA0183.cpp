#include "NMEA0183.h"
#include "config.h"


void addNMEAChecksumToLine(const char *baseLine, char *outputLine) 
{
    int checksum = 0;
    for (int i = 1; baseLine[i] != 0; i++)
        checksum ^= baseLine[i];
    sprintf(outputLine, "%s*%02X\r\n", baseLine, checksum);
}
// --- 3b. GENERATOR PAKIETÓW NMEA (Standard GPS / Instrumenty) --- (teraz przyjmuje YachtState)
void generateNmeaSentences(char *pack, int &offset, int packetCounter, struct tm *timeinfo, const YachtState &snap)
{
  char timeBuf[16], dateBuf[8];
  strftime(timeBuf, sizeof(timeBuf), "%H%M%S.00", timeinfo);
  strftime(dateBuf, sizeof(dateBuf), "%d%m%y", timeinfo);

  char baseLine[150], finalLine[160];
  char latB[20], lonB[20];

  sprintf(latB, "%02d%07.4f", (int)fabsf(snap.lat), (fabsf(snap.lat) - (int)fabsf(snap.lat)) * 60.0f);
  sprintf(lonB, "%03d%07.4f", (int)fabsf(snap.lon), (fabsf(snap.lon) - (int)fabsf(snap.lon)) * 60.0f);

  // ================================================================
  // === GRUPA FAST – wysyłana co sendInterval sekund (z WWW) ===
  // ================================================================

  // $GPRMC – Najważniejsza ramka pozycyjna (OpenCPN używa jej jako głównej)
  // Zawiera: czas, status, pozycja, SOG, COG, data, wariacja magnetyczna
  sprintf(baseLine, "$GPRMC,%s,A,%s,N,%s,E,%.1f,%.1f,%s,5.0,E", timeBuf, latB, lonB, snap.sog, snap.cog, dateBuf);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // $GPGGA – Jakość fixu GPS (pasek satelitów + zielona/czerwona łódka)
  // Zawiera: czas, pozycja, jakość fixu, liczba satelitów, HDOP, wysokość
  sprintf(baseLine, "$GPGGA,%s,%s,N,%s,E,1,12,1.0,%.1f,M,0.0,M,,", timeBuf, latB, lonB, snap.depth);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // $GPVTG – Kurs i prędkość (COG + SOG w węzłach i km/h)
  // OpenCPN bardzo lubi tę ramkę do wyświetlania wektora prędkości
  sprintf(baseLine, "$GPVTG,%.1f,T,%.1f,M,%.1f,N,%.1f,K", snap.cog, snap.cog, snap.sog, snap.sog * 1.852f);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // $IIHDT – Heading prawdziwy (kurs kompasu / gyro)
  // Z leeway (dryf wiatrowy) – heading ≠ COG jak w prawdziwym jachcie
  // Heading jest teraz aktualizowany w updateYachtPhysics
  // Jeśli heading jest 0 i jacht się nie rusza, wysyłamy 0.0, inaczej aktualny heading
  float hdt_heading = snap.heading;
  if (snap.sog < 0.5 && snap.heading == 0.0)
    hdt_heading = 0.0; // Jeśli stoi i heading 0, to 0

  sprintf(baseLine, "$IIHDT,%.1f,T", hdt_heading);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // $WIMWV – Wiatr pozorny (Apparent Wind) – najważniejszy dla żeglarzy
  // Kąt i prędkość wiatru względem jachtu
  sprintf(baseLine, "$WIMWV,%.1f,R,%.1f,N,A", snap.awa, snap.aws);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // --- Obliczenie i wysyłka TWA (True Wind Angle) ---
  // $WIMWV z flagą 'T' oznacza wiatr rzeczywisty względem osi jachtu
  float twa_nmea = snap.twa; // Używamy obliczonego TWA ze snapshotu
  // NMEA oczekuje kąta w zakresie 0-359.9
  if (twa_nmea < 0)
    twa_nmea += 360.0f;
  sprintf(baseLine, "$WIMWV,%.1f,T,%.1f,N,A", twa_nmea, snap.tws);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // $VHW – Prędkość przez wodę (Speed Through Water) + heading
  // Dashboard pokazuje różnicę STW vs SOG (prąd)
  sprintf(baseLine, "$VHW,%.1f,T,,M,%.1f,N,,K", snap.cog, snap.sog);
  addNMEAChecksumToLine(baseLine, finalLine);
  safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

  // ================================================================
  // === GRUPA MEDIUM – co 2 × sendInterval ===
  // ================================================================
  if (packetCounter % 2 == 0)
  {

    // $SDDPT – Głębokość wody (Depth)
    sprintf(baseLine, "$SDDPT,%.1f,0.0", snap.depth);
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

    // $WIMWD – Wiatr rzeczywisty (True Wind Direction + Speed)
    // Bardzo lubiany w instrumentach wiatrowych OpenCPN
    sprintf(baseLine, "$WIMWD,%.1f,T,%.1f,M,%.1f,N,%.1f,M", snap.twd, snap.twd, snap.tws, snap.tws * 0.5144f);
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

    // $IIHDG – Heading magnetyczny + dewiacja + wariacja
    sprintf(baseLine, "$IIHDG,%.1f,,,%.1f,E", snap.cog, 5.0);
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);
  }

  // ================================================================
  // === GRUPA SLOW – co 5 × sendInterval ===
  // ================================================================
  if (packetCounter % 5 == 0)
  {

    // $IIVLW – Log (przebyta droga przez wodę) – total + trip
    sprintf(baseLine, "$IIVLW,%.1f,N,%.1f,N", snap.total_log, snap.trip_log);
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

    // $IIRPM – Obroty silnika (Engine RPM)
    sprintf(baseLine, "$IIRPM,E,1,%.0f,,A", (double)snap.rpm);
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

    // $IIXDR – Dane dodatkowe (RPM dla OpenCPN - zwykły RPM nie działa
    if (snap.rpm > 0.0f)
    {
      sprintf(baseLine, "$IIXDR,T,%.1f,R,ENGINE#0", (double)snap.rpm);
    }
    else
    {
      sprintf(baseLine, "$IIXDR,T,0.0,R,ENGINE#0");
    }
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);

    // $GPZDA – Dokładny czas i data UTC (do synchronizacji OpenCPN)
    sprintf(baseLine, "$GPZDA,%s,%02d,%02d,%04d,,",
            timeBuf, timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
    addNMEAChecksumToLine(baseLine, finalLine);
    safeAppend(pack, offset, NMEA_BUFFER_SIZE, finalLine);
  }
}