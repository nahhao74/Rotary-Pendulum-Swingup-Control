#include "encoder.h"
#include "config.h"
#include <math.h>

volatile long g_pulse_cart = 0;
volatile long g_pulse_pen  = 0;

static void IRAM_ATTR encoderISR_cart() {
  if (digitalRead(PIN_PHASE_A_CART) == digitalRead(PIN_PHASE_B_CART)) g_pulse_cart--;
  else g_pulse_cart++;
}

static void IRAM_ATTR encoderISR_pen() {
  if (digitalRead(PIN_PHASE_A_PEN) == digitalRead(PIN_PHASE_B_PEN)) g_pulse_pen--;
  else g_pulse_pen++;
}

void encoder_init() {
  pinMode(PIN_PHASE_A_CART, INPUT_PULLUP);
  pinMode(PIN_PHASE_B_CART, INPUT_PULLUP);
  pinMode(PIN_PHASE_A_PEN,  INPUT_PULLUP);
  pinMode(PIN_PHASE_B_PEN,  INPUT_PULLUP);

  attachInterrupt(PIN_PHASE_A_CART, encoderISR_cart, CHANGE);
  attachInterrupt(PIN_PHASE_A_PEN,  encoderISR_pen,  CHANGE);
}

void encoder_read_angles(float* theta_rad, float* alpha_rad, float* alpha_deg_out) {
  // theta (cart/arm)
  const float theta = (g_pulse_cart / ENCODER_PPR) * 2.0f * PI;

  // alpha (pendulum) -> wrap + zero at top
  float angle_pen_deg = (g_pulse_pen / ENCODER_PPR) * 360.0f;
  angle_pen_deg = fmodf(angle_pen_deg, 360.0f);
  if (angle_pen_deg < 0.0f) angle_pen_deg += 360.0f;

  // 0 gốc là dưới -> trừ 180 để 0 là đỉnh
  float alpha_deg = angle_pen_deg - 180.0f;

  // Wrap về [-180, 180]
  if (alpha_deg > 180.0f)  alpha_deg -= 360.0f;
  if (alpha_deg < -180.0f) alpha_deg += 360.0f;

  const float alpha = alpha_deg * (PI / 180.0f);

  if (theta_rad) *theta_rad = theta;
  if (alpha_rad) *alpha_rad = alpha;
  if (alpha_deg_out) *alpha_deg_out = alpha_deg;
}
