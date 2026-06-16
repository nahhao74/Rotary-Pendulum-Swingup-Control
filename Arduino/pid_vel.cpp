#include "pid_vel.h"
#include "config.h"
#include <math.h>


static float s_integral = 0.0f;
static float s_prev_err = 0.0f;

void pid_vel_init() {
  pid_vel_reset();
}

void pid_vel_reset() {
  s_integral = 0.0f;
  s_prev_err = 0.0f;
}

float pid_vel_compute(float target_rpm, float current_rpm, float dt) {
  // 1. Tính sai số
  float err = target_rpm - current_rpm;

  // 2. Tích phân 
  s_integral += err * dt;
  
  // Anti-windup (Giới hạn tích phân để tránh quá bão hòa)
  float i_limit = 255.0f / VEL_KI;
  if (s_integral > i_limit) s_integral = i_limit;
  if (s_integral < -i_limit) s_integral = -i_limit;

  // 3. Đạo hàm 
  float derivative = (err - s_prev_err) / dt;
  s_prev_err = err;

  // 4. Tính toán PWM Command 
  float pwm_cmd = (VEL_KP * err) + (VEL_KI * s_integral) + (VEL_KD * derivative);

  // 5. Constrain PWM
  if (pwm_cmd > 255.0f) pwm_cmd = 255.0f;
  if (pwm_cmd < -255.0f) pwm_cmd = -255.0f;

  // 6. CHUYỂN ĐỔI: PWM (-255..255) -> VOLTAGE (-12..12)
  // Để tương thích với hàm motor_set_voltage
  float voltage_out = (pwm_cmd / 255.0f) * V_SUPPLY;

  return voltage_out;
}