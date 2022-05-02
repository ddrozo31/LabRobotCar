/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

 
#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>


#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include "LowPass2ndOrder.h"
#include "SimplePID.h"
#include "MotorClass.h"

// ---------- variable definition

// Pins

const int enca_m1 = 3;//18; //3; // ENCA  14//A pin -> the interrupt pin D5 --- M1
const int encb_m1 = 11;//17; //11; // ENCB  12//B pin -> the digital pin D6 --- M1

const int enca_m2 = 2;//19; //2; // ENCA  14//A pin -> the interrupt pin D5 --- M1
const int encb_m2 = 10;//16; //10; // ENCB  12//B pin -> the digital pin D6 --- M1

void get_posi();
void readEncoder_m1();
void readEncoder_m2();

volatile int posi_m1;
volatile int posi_m2;

unsigned long delta_time;
unsigned long previous_time = 0;
unsigned long sample_time = 100;

float target_rw = 0.0;
float target_lw = 0.0;

// -- print data structure
float data[7];

motor_class motor_lw;
motor_class motor_rw;

//====================== ROS =================

// ros obj
ros::NodeHandle  nh;


// ========================= rwheel sub and pub
// publishers
// msg ros variables publisher 
std_msgs::Float64 rwheel_speed;
ros::Publisher rwheel_speed_obj("/rwheel_speed", &rwheel_speed);

// msg ros variables publisher 
//std_msgs::Float64 rwheel_ticks;
//ros::Publisher rwheel_ticks_obj("/rwheel_ticks", &rwheel_ticks);

// subscriber
std_msgs::Float64 rwheel_speed_target;
void rwheel_spd_cmdCb(const std_msgs::Float64 &rwheel_speed_msg){
  target_rw = rwheel_speed_msg.data;
}
ros::Subscriber<std_msgs::Float64> rwheel_spd_tgt("/rwheel_speed_target", &rwheel_spd_cmdCb);


// ========================= lwheel sub and pub
// publishers
// msg ros variables publisher 
std_msgs::Float64 lwheel_speed;
ros::Publisher lwheel_speed_obj("/lwheel_speed", &lwheel_speed);

// msg ros variables publisher 
//std_msgs::Float64 lwheel_ticks;
//ros::Publisher lwheel_ticks_obj("/lwheel_ticks", &lwheel_ticks);

// subscriber
std_msgs::Float64 lwheel_speed_target;
void lwheel_spd_cmdCb(const std_msgs::Float64 &lwheel_speed_msg){
  target_lw = lwheel_speed_msg.data;
}
ros::Subscriber<std_msgs::Float64> lwheel_spd_tgt("/lwheel_speed_target", &lwheel_spd_cmdCb);


void setup()
{
  //Serial.begin(115200);
  //Serial.println();

  //motor1.setParams(enca, encb, pwm, ina, inb, en, kp, ki, kd)
  
  motor_lw.setParams(enca_m1, encb_m1, 5, 7, 8, A0, 0.45, 1.2, 0.015);
  motor_rw.setParams(enca_m2, encb_m2, 6, 4, 9, A1, 0.45, 1.2, 0.015);
  //

  attachInterrupt(digitalPinToInterrupt(enca_m1),readEncoder_m1,RISING);
  attachInterrupt(digitalPinToInterrupt(enca_m2),readEncoder_m2,RISING);

  nh.initNode();

  nh.advertise(rwheel_speed_obj);
  nh.advertise(lwheel_speed_obj);
  
  //nh.advertise(rwheel_ticks_obj);
  nh.subscribe(rwheel_spd_tgt);
  nh.subscribe(lwheel_spd_tgt);

}


void loop()
{
  unsigned long current_time = millis();
  delta_time = current_time - previous_time;
   
  if (delta_time > sample_time)
  {    
    if (nh.connected()) {
      get_posi();
  
     
      motor_lw.get_speed(data[5]); 
      
      //lwheel_ticks.data = data[2];target_rw
      //lwheel_ticks_obj.publish( &lwheel_ticks);
      
      lwheel_speed.data = data[5];
      lwheel_speed_obj.publish( &lwheel_speed);
      
      motor_lw.motor_update(target_lw);
      //motor1.set_motor(1.0, 50.0);
        
  
  
      motor_rw.get_speed(data[3]); 
  
      //rwheel_ticks.data = data[2];
      //rwheel_ticks_obj.publish( &rwheel_ticks);
      rwheel_speed.data = data[3];
      rwheel_speed_obj.publish( &rwheel_speed);
  
      motor_rw.motor_update(target_rw);
  
      
      char log_msg[16];
      char target_str[8];   
      
      dtostrf(target_rw, 4, 4, target_str); // Leave room for too large numbers!
      sprintf(log_msg, "tgt_rw: %s",target_str);
      nh.loginfo(log_msg);
  
    
      dtostrf(target_lw, 4, 4, target_str); // Leave room for too large numbers!
      sprintf(log_msg, "tgt_lw: %s",target_str);
      nh.loginfo(log_msg);
     
    }else
    {
      motor_lw.set_motor(0.0, 0.0);
      motor_rw.set_motor(0.0, 0.0);
    }

    previous_time = current_time;
    nh.spinOnce();
  }
}

void get_posi()
{
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      motor_lw.set_ticks(posi_m1);
      motor_rw.set_ticks(posi_m2);
   }
}

void readEncoder_m1(){
  int b = digitalRead(encb_m1);
  if(b > 0){
    posi_m1++;
  }
  else{
    posi_m1--;
  }
}

void readEncoder_m2(){
  int b = digitalRead(encb_m2);
  if(b > 0){
    posi_m2++;
  }
  else{
    posi_m2--;
  }
}
