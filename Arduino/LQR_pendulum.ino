#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "lqr_controller.h"
#include "pid_vel.h"
#include "logger.h"

float g_rpm_cart = 0.0f;
float g_prev_theta = 0.0f;
float g_prev_alpha = 0.0f;
float g_alpha_dot = 0.0f;
float u_lqr = 0.0f;

void setup() {
  Serial.begin(SERIAL_BAUD);
  encoder_init();
  motor_init();
  lqr_init();
  pid_vel_init();
  logger_init();
  delay(100);
  Serial.println("SYSTEM_READY");
}

void loop() {
  static unsigned long last_time_ms = 0;
  static bool last_was_balancing = false; 
  
  // Vẫn giữ gốc ảo để không bị giật ngang khi bắt
  static float theta_offset = 0.0f;    

  unsigned long now = millis();

  if ((now - last_time_ms) >= (unsigned long)(DT_S * 1000.0f)) {
    float dt_real = (now - last_time_ms) / 1000.0f;
    last_time_ms = now;

    // --- 1. ĐỌC CẢM BIẾN ---
    float theta = 0.0f, alpha = 0.0f, alpha_deg = 0.0f;
    encoder_read_angles(&theta, &alpha, &alpha_deg);

    float alpha_diff = alpha - g_prev_alpha;
    if (alpha_diff > PI) alpha_diff -= 2.0f * PI;
    if (alpha_diff < -PI) alpha_diff += 2.0f * PI;
    g_alpha_dot = VEL_FILTER_CONST * (alpha_diff / dt_real) + (1.0f - VEL_FILTER_CONST) * g_alpha_dot;
    g_prev_alpha = alpha;

    float theta_diff = theta - g_prev_theta;
    g_rpm_cart = VEL_FILTER_CONST * ((theta_diff / (2.0f * PI)) / dt_real * 60.0f) + (1.0f - VEL_FILTER_CONST) * g_rpm_cart;
    g_prev_theta = theta;

    // --- 2. TÍNH TOÁN ĐIỀU KHIỂN ---
    float target_rpm = 0.0f;
    float V_final = 0.0f;

    // VÙNG BẮT CHUẨN 20 ĐỘ (~0.35 Radian)
    const float ALPHA_CATCH = 0.35f;      
    const float OMEGA_CATCH_LIMIT = 15.0f; 

    bool should_balance = (fabs(alpha) < ALPHA_CATCH && fabs(g_alpha_dot) < OMEGA_CATCH_LIMIT);
    
    if (should_balance) { 
        if (!last_was_balancing) {
            theta_offset = theta;      
            pid_vel_reset();           
            Serial.println("MODE: BALANCING_ACTIVE (FULL POWER)"); 
            last_was_balancing = true;
        }

        float theta_relative = theta - theta_offset;
        u_lqr = lqr_step(theta, alpha, dt_real);
        target_rpm = (u_lqr / MOTOR_KM) * 9.5493f; 

    } else {
        if (last_was_balancing) {
            Serial.println("MODE: SWING_UP");
            last_was_balancing = false;
        }

        // --- CHẾ ĐỘ SWING-UP ---
        float gain_reduction = 0.2f + 0.8f * fabs(sinf(alpha)); 
        
        
        
        target_rpm = (-K_PUMP * gain_reduction * g_alpha_dot) - (K_CENTER * theta);


  
        const float THETA_BRAKE_LIMIT = 1.0f; 
        if (theta > THETA_BRAKE_LIMIT && target_rpm > 0.0f) target_rpm = -80.0f; 
        else if (theta < -THETA_BRAKE_LIMIT && target_rpm < 0.0f) target_rpm = 80.0f;

        target_rpm = constrain(target_rpm, -250.0f, 250.0f);
    }

    // --- 3. OUTPUT ---
    target_rpm = constrain(target_rpm, -RPM_LIMIT, RPM_LIMIT);
    V_final = pid_vel_compute(target_rpm, g_rpm_cart, dt_real);
    motor_set_voltage(V_final);
    int pwm_val = (int)(fabsf(V_final) / V_SUPPLY * (float)MAX_PWM);

    // --- 4. LOGGING ---
    Serial.print(now / 1000.0f); Serial.print(",");
    Serial.print(alpha_deg);     Serial.print(",");
    Serial.print(target_rpm);    Serial.print(",");
    Serial.print(theta);         Serial.print(",");
    Serial.print(u_lqr);         Serial.print(",");
    Serial.print(pwm_val);         Serial.print(",");
    Serial.println(g_rpm_cart);
  }
}