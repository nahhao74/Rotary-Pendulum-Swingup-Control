#pragma once
#include <Arduino.h>

void logger_init();

// Print: time(s), Vm(V), alpha(deg), theta(deg)
void logger_print(unsigned long now_ms, float Vm, float alpha_deg, float theta_rad);
