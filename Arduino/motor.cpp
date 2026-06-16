#include "motor.h"
#include "config.h"
#include <math.h>

// NOTE:


void motor_init() {
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);

  ledcAttach(PIN_R_PWM, 5000, 8);
  ledcAttach(PIN_L_PWM, 5000, 8);
  ledcWrite(PIN_R_PWM, 0);
  ledcWrite(PIN_L_PWM, 0);
}

void motor_set_voltage(float voltage) {
  // Saturation
  if (voltage > VM_LIMIT)  voltage = VM_LIMIT;
  if (voltage < -VM_LIMIT) voltage = -VM_LIMIT;

  int pwm_val = (int)(fabsf(voltage) / V_SUPPLY * (float)MAX_PWM);
  if (pwm_val > MAX_PWM) pwm_val = MAX_PWM;

  if (voltage > 0.0f) {
    ledcWrite(PIN_R_PWM, pwm_val);
    ledcWrite(PIN_L_PWM, 0);
  } else {
    ledcWrite(PIN_R_PWM, 0);
    ledcWrite(PIN_L_PWM, pwm_val);
  }
}
