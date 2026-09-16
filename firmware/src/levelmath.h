#pragma once
#include <stdint.h>

// Pure math, no Arduino dependencies, so it can be unit tested on the host.
// These are the calculations that flood a tank or mis-report a fill when wrong.

namespace wiot {

inline bool validDistance(uint16_t cm, uint16_t minCm, uint16_t maxCm) {
  return cm >= minCm && cm <= maxCm;
}

// Ultrasonic sits above the water looking down, so distance shrinks as the
// tank fills: emptyCm is the far reading (floor), fullCm the near one.
inline float levelPercent(uint16_t distanceCm, uint16_t emptyCm, uint16_t fullCm) {
  int32_t span = (int32_t)emptyCm - (int32_t)fullCm;
  if (span < 1) span = 1;                       // misconfigured; don't divide by zero
  float pct = 100.0f * ((int32_t)emptyCm - (int32_t)distanceCm) / (float)span;
  if (pct < 0.0f)   return 0.0f;
  if (pct > 100.0f) return 100.0f;
  return pct;
}

inline uint32_t litresFromPercent(float pct, uint32_t fullLitres) {
  if (pct <= 0.0f) return 0;
  if (pct >= 100.0f) return fullLitres;
  return (uint32_t)(pct / 100.0f * fullLitres);
}

inline float lpmFromPulses(uint32_t pulsesPerSecond, float pulsesPerLitre) {
  if (pulsesPerLitre < 0.1f) return 0.0f;       // guard a bad K-factor from MQTT
  return (pulsesPerSecond / pulsesPerLitre) * 60.0f;
}

// Median of 5 rejects the side echoes a 3m sump throws off its walls.
// Failed reads arrive as 0 and sort to the front, so the median is only 0
// when three or more of the five samples failed.
inline uint16_t medianOf5(uint16_t s[5]) {
  for (int i = 1; i < 5; i++) {
    uint16_t v = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  return s[2];
}

}  // namespace wiot
