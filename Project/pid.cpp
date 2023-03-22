#include "pid.h"

pid::pid(float h_, float K_, float b_, float Ti_, float Td_, float N_) 
  : h {h_}, K {K_}, b {b_}, Ti {Ti_}, Td {Td_}, N {N_}, I {0.0}, D {0.0}, y_old {0.0}
{
  
}

float pid::compute_control(float r, float y) 
{
  float P = K*(b*r - y);
  float ad = Td / (Td + N*h);
  float bd = Td*K*N / (Td + N*h);
  D = ad*D - bd*(y - y_old);
  float u = P + I + D;
  if (u < 0) u = 0;
  if (u > 4095) u = 4095;
  return u;
}
