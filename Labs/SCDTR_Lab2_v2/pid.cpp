#include "pid.h"

pid::pid(float h_, float K_, float b_, float Ti_, float Td_, float N_, float Tt_) 
  : h {h_}, K {K_}, b {b_}, Ti {Ti_}, Td {Td_}, 
  N {N_}, I {0.0}, D {0.0}, y_old {0.0}, Tt {Tt_}, 
  es {0.0}, K_old {0.0}, b_old {0.0}, in_transition {false}
{
  
}

float pid::compute_control(float r, float y) 
{
  float P = K*(b*r - y);
  float ad = Td / (Td + N*h);
  float bd = Td*K*N / (Td + N*h);
  if (in_transition) {
    I = I + K_old*(b_old*r - y) - K*(b*r - y);
    in_transition = false;
  }
  D = ad*D - bd*(y - y_old);
  float v = P + I + D;
  float u = v;
  if (v < 0) u = 0;
  if (v > 4095) u = 4095;
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
