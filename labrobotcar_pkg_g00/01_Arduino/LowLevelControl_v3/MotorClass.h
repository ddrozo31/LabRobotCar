
#ifndef MotorClass_h
#define MotorClass_h
#include <Arduino.h>
#include "LowPass2ndOrder.h"
#include "SimplePID.h"

class motor_class{
  private:

    int enca; // 
    int encb; // 
    int pwm;  // 
    int ina;  // 
    int inb;
    int en;


  
    float umax = 255;
    //float target = 20.0;

    // gloabl variables
    long prevT = 0;
    long currT = 0;
    float deltaT = 0;
    int posPrev;
    volatile int pos;


    LowPass2ndOrder lp_m1;
    LowPass2ndOrder lp_ticks;
    
    // PID Simple instances f0(3),fs(1e3),adp(true)
    SimplePID pid;


    float pwr = 0.0;
    float dir = 0.0;

    int vel_rpm_raw = 0;
    int pos_filt = 0;

    float vel;
    float vel_rpm;
    
    // -- ticks to rpm
    const int ppr = 16; // ticks per rotation
    const int gear_ratio = 50; //
    // X2 encoding, in which both the rising and falling edges of channel A are counted.
    const int decoder_number = 1;
    // X4 encoding, in which both the rising and falling edges of channels A and B are counted.
    // const int decoder_number = 4;
    


  public:
    // constructor
    motor_class();
  
    // A function to set the parameters
    void setParams(int encaIn,int encbIn,int pwmIn,int inaIn,int inbIn,int enIn, float kpIn, float kiIn, float kdIn);
  
    void set_motor(float dir, float pwm_val);
    
    void encoder2speed();

    void get_speed(float &spd);

    void set_ticks(int ticks);

    void get_ticks(float &tck);
    
    void motor_update(float target);
};

#endif
