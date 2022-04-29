/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;


void messageCb( const std_msgs::Int32& toggle_msg){
  digitalWrite(13, toggle_msg.data);   // blink the led
}

ros::Subscriber<std_msgs::Int32> sub("toggle_led", messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[] = "Hello World ROS!";

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
