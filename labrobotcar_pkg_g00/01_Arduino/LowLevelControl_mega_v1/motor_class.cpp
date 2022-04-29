#include "motor_class.h"
//SimplePID::SimplePID(): kp(1),kd(0),ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

motor_class::motor_class():enca(0),encb(0),pwm(0),ina(0),inb(0),en(0){}

// A function to set the parameters
void motor_class::setParams(int encaIn,int encbIn,int pwmIn,int inaIn,int inbIn,int enIn, float kpIn, float kiIn, float kdIn)
{
  enca = encaIn;
  encb = encbIn;
  pwm = pwmIn;
  ina = inaIn;
  inb = inbIn;
  en =  enIn;


  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  
  pinMode(pwm, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(en, OUTPUT);

  digitalWrite(en, HIGH); 

  lp_m1.init(3,1e3,true);
  lp_ticks.init(3,1e3,true);

  //pid.setParams(2.5,20,0.05,255);
  pid.setParams(kpIn,kiIn,kdIn,umax);
}

void motor_class::set_motor(float dir, float pwm_val)
{
  analogWrite(pwm,pwm_val);
  if(dir == 1){
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);    
  }
  else if (dir == -1)
  {
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);   
  }
  else
  {
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW); 
  }
}

void motor_class::encoder2speed()
{

  currT = micros();
  deltaT = (float)(currT-prevT)/1.0e6;
  prevT = currT; // current time
     
  pos_filt = lp_ticks.filt(pos);

  vel = ((pos - posPrev)/deltaT);

  // storage values
  posPrev = pos; // current tick 
  // vel in rpm
  vel_rpm_raw = ((vel)/(ppr * decoder_number * gear_ratio))*(60);
  vel_rpm = lp_m1.filt(vel_rpm_raw);
}

void motor_class::set_ticks(int ticks)
{
  pos = ticks;
}

void motor_class::motor_update(float target)
{ 
  
  encoder2speed();
  //target = 20*(sin(currT/1e6)>0);
  
  // evaluate the control signal
  pid.evalu(vel_rpm,target, deltaT, pwr, dir);  
  
  set_motor(dir, pwr);
}

void  motor_class::get_speed(float &spd)
{
  spd = vel_rpm;
}

void  motor_class::get_ticks(float &tck)
{
  tck = pos_filt;
}
