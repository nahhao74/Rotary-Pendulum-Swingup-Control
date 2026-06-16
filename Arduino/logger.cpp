#include "logger.h"
#include <Arduino.h>
#include <math.h>

void logger_init() {
  // nothing for now
}

void logger_print(unsigned long now_ms, float Vm, float alpha_deg, float theta_rad) {
  Serial.print(now_ms / 1000.0f); Serial.print(",");
  Serial.print(Vm);               Serial.print(",");
  Serial.print(alpha_deg);        Serial.print(",");
  Serial.println(theta_rad * 180.0f / PI);
}
