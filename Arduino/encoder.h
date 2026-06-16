#pragma once
#include <Arduino.h>

// Encoders pulse counters
extern volatile long g_pulse_cart;
extern volatile long g_pulse_pen;

// Init pins + interrupts
void encoder_init();

// Read angles (theta, alpha in rad) + alpha_deg for logging
void encoder_read_angles(float* theta_rad, float* alpha_rad, float* alpha_deg_out);
