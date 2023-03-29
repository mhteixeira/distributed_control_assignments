#include "pid.h"

pid::pid(float h_, float K_, float b_, float Ti_, float Td_, float N_, float Tt_)
    : h{h_}, K{K_}, b{b_}, Ti{Ti_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, y_old{0.0}, Tt{Tt_},
      es{0.0}, K_old{0.0}, b_old{0.0}, in_transition{false},
      anti_windup_status{true}, feedforward_status{true}
{
}

float pid::compute_control(float r, float y)
{
  float P = K * ((feedforward_status * (b - 1) + 1) * r - y);
  float ad = Td / (Td + N * h);
  float bd = Td * K * N / (Td + N * h);

  // Procedure to update the integrator
  // to create bumpless transfer
  if (in_transition)
  {
    I = I + K_old * (b_old * r - y) - K * (b * r - y);
    in_transition = false;
  }

  D = ad * D - bd * (y - y_old);
  float v = P + I + D;
  float u = v;
  if (v < 0)
    u = 0;
  if (v > 255)
    u = 255;
  es = u - v;
  return u;
}

void pid::change_parameters(float K_new, float b_new)
{
  K_old = K;
  b_old = b;
  K = K_new;
  b = b_new;
  in_transition = true;
}

void pid::set_anti_windup_status(bool new_status)
{
  anti_windup_status = new_status;
}

bool pid::get_anti_windup_status()
{
  return anti_windup_status;
}

void pid::set_feedforward_status(bool new_status)
{
  feedforward_status = new_status;
}

bool pid::get_feedforward_status()
{
  return feedforward_status;
}