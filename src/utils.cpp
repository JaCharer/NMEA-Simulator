
#include <Arduino.h>
#include <time.h>
#include "config.h"

// Pomocnicza funkcja do bezpiecznego dopisywania do bufora
void safeAppend(char *buffer, int &offset, int maxSize, const char *line)
{
  if (offset >= maxSize - 1)
    return;
  int written = snprintf(buffer + offset, maxSize - offset, "%s", line);
  if (written > 0 && offset + written < maxSize)
  {
    offset += written;
  }
}

// Funkcja pomocnicza do pobierania timestampu
void getTimestamp(char* dest, size_t size)
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10))
  {
    snprintf(dest, size, "[SYSTEM]");
    return;
  }
  snprintf(dest, size, "[%04d-%02d-%02d %02d:%02d:%02d]",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}