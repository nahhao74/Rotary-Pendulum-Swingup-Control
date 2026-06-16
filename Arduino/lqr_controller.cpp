#include "lqr_controller.h"
#include "config.h"
#include <math.h>

unsigned long g_last_time_ms = 0;

// state / derivative
static float s_theta_dot = 0.0f;
static float s_alpha_dot = 0.0f;
static float s_prev_theta = 0.0f;
static float s_prev_alpha = 0.0f;

void lqr_init() {
  s_theta_dot = 0.0f;
  s_alpha_dot = 0.0f;
  s_prev_theta = 0.0f;
  s_prev_alpha = 0.0f;
}

static inline float lpf(float a, float v_new, float v_old) {
  return a * v_new + (1.0f - a) * v_old;
}

float lqr_step(float theta, float alpha, float dt_real) {
  // Derivative
  const float theta_dot_new = (theta - s_prev_theta) / dt_real;
  const float alpha_dot_new = (alpha - s_prev_alpha) / dt_real;

  s_theta_dot = lpf(VEL_FILTER_CONST, theta_dot_new, s_theta_dot);
  s_alpha_dot = lpf(VEL_FILTER_CONST, alpha_dot_new, s_alpha_dot);

  s_prev_theta = theta;
  s_prev_alpha = alpha;

  float Vm = 0.0f;

  // Safety: only enable near top
  if (fabsf(alpha) < ALPHA_ENABLE) {
    Vm = - ( K_THETA * theta
           + K_ALPHA * alpha
           + K_THETA_DOT * s_theta_dot
           + K_ALPHA_DOT * s_alpha_dot );
  } else {
    Vm = 0.0f;
  }
  return Vm;
}
