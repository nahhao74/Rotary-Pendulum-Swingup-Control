#pragma once
#include <Arduino.h>

// last loop timestamp, kept in main for timing
extern unsigned long g_last_time_ms;

void lqr_init();

// returns Vm (Volt)
float lqr_step(float theta, float alpha, float dt_real);
