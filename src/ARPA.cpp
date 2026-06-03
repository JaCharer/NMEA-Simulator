#include <Arduino.h>
#include <math.h>
#include "ARPA.h"


// =========================================================================
// MATEMATYKA ARPA (Przewidywanie Kolizji: CPA / TCPA)
// Oblicza wektory i zwraca TRUE, jeśli przecinamy się w odległości < 2 NM,
// a czas zderzenia (TCPA) to od 0 do 15 minut.
// =========================================================================
bool checkCollisionRisk_ARPA(float lat1, float lon1, float sog1, float cog1,
                             float lat2, float lon2, float sog2, float cog2)
{

  // 1. Konwersja współrzędnych na lokalny układ kartezjański (XY) w milach morskich.
  // Jacht jest punktem odniesienia (0,0).
  float dx_nm = (lon2 - lon1) * cosf(lat1 * PI / 180.0) * 60.0;
  float dy_nm = (lat2 - lat1) * 60.0;

  // 2. Rozkład wektorów prędkości obu jednostek na składowe X i Y (Węzły = NM/h).
  float v1_x = sog1 * sin(cog1 * PI / 180.0);
  float v1_y = sog1 * cos(cog1 * PI / 180.0);
  float v2_x = sog2 * sin(cog2 * PI / 180.0);
  float v2_y = sog2 * cos(cog2 * PI / 180.0);

  // 3. Obliczenie wektora prędkości względnej (Relative Velocity).
  float dv_x = v2_x - v1_x;
  float dv_y = v2_y - v1_y;
  float v_rel2 = dv_x * dv_x + dv_y * dv_y;

  if (v_rel2 < 0.01)
    return false; // Statki stoją w miejscu względem siebie

  // 4. TCPA (Time to Closest Point of Approach) - czas do najbliższego zbliżenia.
  // Ujemny wynik oznacza, że statki już się mijają.
  float tcpa_hours = -(dx_nm * dv_x + dy_nm * dv_y) / v_rel2;
  float tcpa_min = tcpa_hours * 60.0;

  if (tcpa_min < 0.0 || tcpa_min > 15.0)
    return false;

  // 5. CPA (Closest Point of Approach) - najmniejsza przewidywana odległość między statkami.
  float cpa_x = dx_nm + dv_x * tcpa_hours;
  float cpa_y = dy_nm + dv_y * tcpa_hours;
  float cpa_nm = sqrtf(cpa_x * cpa_x + cpa_y * cpa_y);

  if (cpa_nm < 2.0)
  {
    return true;
  }

  return false;
}
