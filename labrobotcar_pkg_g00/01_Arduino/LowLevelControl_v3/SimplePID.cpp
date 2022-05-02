#include "SimplePID.h"

SimplePID::SimplePID(): kp(1),kd(0),ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

// A function to set the parameters
void SimplePID::setParams(float kpIn, float kiIn, float kdIn, float umaxIn){
  kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn; 
}

// function to compute the control law
void SimplePID::evalu(float value, float target, float deltaT, float &pwr, float &dir){
  // error signal
  float e = target - value;
  
  // derivative error
  float dedt = (e-eprev)/deltaT;
  
  // integral error
  eintegral = eintegral +e*deltaT;

  // control law  
  float u = kp*e + ki*eintegral + kd*dedt;
  
  // set the motor direction
  dir = 1;
  if (u<0) {
    dir = -1;
  }
  
  //set the motor speed and
  pwr = abs(u);
  if (pwr > umax){
    pwr = umax;
  }
  
  if (target == 0){
    pwr = 0;
  }

/*
  int flag1, flag2;
  
  if (u != pwr)
  {
    flag1 = 1;
  }else
  {
    flag1 = 0;
  }

  if ((u > 0 && e > 0)  || (u<0 && e < 0))
  {
    flag2 = 1;
  }else
  { 
    flag2 = -1;
  }

  if (flag1 && flag2)
  {
    eintegral = 0;
  }
*/
  // store previous error
  eprev = e;
}
