#pragma once
#include <Arduino.h>
#include <math.h>

// ================= Serial =================
static const uint32_t SERIAL_BAUD = 115200;

// ================= CẤU HÌNH PHẦN CỨNG =================
static const uint8_t PIN_R_PWM       = 25; 
static const uint8_t PIN_L_PWM       = 26;
static const uint8_t PIN_PHASE_A_CART= 34;
static const uint8_t PIN_PHASE_B_CART= 35;
static const uint8_t PIN_PHASE_A_PEN = 18;
static const uint8_t PIN_PHASE_B_PEN = 19;

// ================= GAIN LQR (OUTER LOOP) =================
// --- BỘ HỆ SỐ CHỐNG TRÔI TAY QUAY ---
// --- BỘ HỆ SỐ CATCH CỰC GẮT (BEAST MODE) ---
static const float K_THETA     =  -4.0000;  
static const float K_ALPHA     =  95.0000;  
static const float K_THETA_DOT =  -4.5000;  
static const float K_ALPHA_DOT =  18.5000;  

// ================= PID VELOCITY (INNER LOOP) =================

// static const float VEL_KP = 0.27054;
// static const float VEL_KI = 1.60388;
// static const float VEL_KD = 0.00039;

static const float VEL_KP = 0.1892f;
static const float VEL_KI = 1.4834f;
static const float VEL_KD = 0.0003f;

// ================= MAPPING PARAMS =================
// Hệ số Back-EMF (V / (rad/s))
// Dùng để đổi từ Volts LQR sang Rad/s, sau đó sang RPM
static const float MOTOR_KM = 0.5f; 

const float K_PUMP = 40.0f;   
const float K_CENTER = 120.0f; 

// ================= SYSTEM PARAMS =================
static const float ENCODER_PPR = 1024.0f; 
static const float V_SUPPLY    = 12.0f;   // Nguồn cấp (
static const int   MAX_PWM     = 255;
static const float DT_S        = 0.01f;   // 10ms 

// ================= SAFETY =================
static const float ALPHA_ENABLE  = 0.35f; // ~20 độ
static const float VM_LIMIT      = 12.0f; // Max Volts
static const float RPM_LIMIT     = 2000.0f;

// ================= FILTERS =================
static const float VEL_FILTER_CONST = 0.6f;