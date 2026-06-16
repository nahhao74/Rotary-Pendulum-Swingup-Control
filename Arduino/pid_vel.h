#pragma once
#include <Arduino.h>

void pid_vel_init();

// Tính toán PID: Input là RPM, Output là Volts (để đưa vào hàm motor_set_voltage)
float pid_vel_compute(float target_rpm, float current_rpm, float dt);

void pid_vel_reset();