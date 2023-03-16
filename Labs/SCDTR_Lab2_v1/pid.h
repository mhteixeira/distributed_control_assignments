#ifndef PID_H
#define PID_H

class pid 
{
  public:
    float I, D, K, Ti, Td, b, h, y_old, N;
    explicit pid(float h_, float K_, float b_, float Ti_, float Td_, float N_);  
    ~pid() {};
    float compute_control(float r, float y);
    void housekeep(float r, float y);
};

inline void pid::housekeep(float r, float y) 
{
  float e = r - y;
  I += K*h/Ti*e;
  y_old = y;  
}

#endif //PID_H
