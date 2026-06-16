#pragma once
#include <Arduino.h>

// Init PWM pins/channels
void motor_init();

// voltage in Volt (will be saturated)
void motor_set_voltage(float voltage);
