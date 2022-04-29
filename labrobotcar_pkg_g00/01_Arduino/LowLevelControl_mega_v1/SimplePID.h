
#ifndef SimplePID_h
#define SimplePID_h
#include <Arduino.h>

class SimplePID{
  private:
    float kp, kd, ki, umax; // PID parameters
    float eprev, eintegral; // error storage

  public:
    // constructor
    SimplePID();
  
    // A function to set the parameters
    void setParams(float kpIn, float kiIn, float kdIn, float umaxIn);
  
    // function to compute the control law
    void evalu(float value, float target, float deltaT, float &pwr, float &dir);
};

#endif
