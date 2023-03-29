#ifndef PID_H
#define PID_H

class pid
{
public:
  float I, D, K, Ti, Td, b, h, y_old, N, Tt, es, K_old, b_old;
  bool in_transition, anti_windup_status, feedforward_status;
  explicit pid(float h_, float K_, float b_, float Ti_, float Td_, float N_, float Tt_);
  ~pid(){};
  float compute_control(float r, float y);
  void change_parameters(float K_new, float b_new);
  void set_anti_windup_status(bool b);
  bool get_anti_windup_status();
  void set_feedforward_status(bool b);
  bool get_feedforward_status();
  void housekeep(float r, float y);
};

inline void pid::housekeep(float r, float y)
{
  float e = r - y;
  // Updating the integrator with
  // anti-windup (back calculation)
  I += K * h / Ti * e + anti_windup_status * h / Tt * es;
  y_old = y;
}

#endif // PID_H
