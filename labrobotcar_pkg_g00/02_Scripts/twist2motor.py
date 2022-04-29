#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


linear_speed = 0.0
angular_speed = 0.0
rwheel_speed = 0.0

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


def diff_drive_ik(x_dot, theta_dot):

    # all in centimeters
    l_between_wheels = 26.6
    e_wheel = 2.5
    d_wheel = 6.7


    b = l_between_wheels-(e_wheel/2)*2
    r = d_wheel/2

    rpm2rads = 2*np.pi/60 

    Ai = np.array([[1/r,0, b/r],
               [1/r,0,-b/r]])

    Bi = np.array([[x_dot],
                  [.0],
                  [theta_dot]])

    wheel_speed = np.matmul(Ai,Bi)*(1/rpm2rads)
    
    return wheel_speed[0,0],wheel_speed[1,0]

def cmd_vel_callback(data):
    global linear_speed, angular_speed

    msg = 'x_dot: {:.3f}, theta_dot: {:.3f}'.format(data.linear.x,data.angular.z)
    linear_speed = data.linear.x
    angular_speed = data.angular.z     


def rwheel_speed_callback(data):
    global rwheel_speed

    msg = 'rwheel_speed: {:.3f}'.format(data.data)
    rwheel_speed = data.data
    #rospy.loginfo(msg)

def twist2motor():
    global linear_speed, angular_speed
    
    rospy.init_node('twist2motor', anonymous=False) #NODE_NAME must change

    cmd_vel_obj = rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)

    rwheel_speed_obj = rospy.Subscriber('/rwheel_speed',Float64,rwheel_speed_callback)

    rwheel_spd_tgt_obj = rospy.Publisher('/rwheel_speed_target',Float64,queue_size=1)

    lwheel_spd_tgt_obj = rospy.Publisher('/lwheel_speed_target',Float64,queue_size=1)

   
    rwheel_tgt_msg = Float64()
    lwheel_tgt_msg = Float64()
   
    rate = rospy.Rate(100) # 10hz

    last_time = rospy.Time.now()
    
    last_rw_spd_tgt = 0.0
    last_lw_spd_tgt = 0.0

    rw_spd_tgt = 0.0
    lw_spd_tgt = 0.0

    rdif = 0.0
    ldif = 0.0

    while not rospy.is_shutdown():

        if linear_speed < -5:
            linear_speed = -5
        elif linear_speed > 5:
            linear_speed = 5

        if angular_speed < -5:
            angular_speed = -5
        elif angular_speed > 5:
            angular_speed = 5



        x_dot_map = map(linear_speed,-10, 10, -70, 70)
        th_dot_map = map(angular_speed, -10, 10, -2.9, 2.9) 


        rw_spd_tgt, lw_spd_tgt = diff_drive_ik(x_dot_map, th_dot_map)

        rwheel_tgt_msg = rw_spd_tgt

        lwheel_tgt_msg = lw_spd_tgt


        if  (last_rw_spd_tgt != rw_spd_tgt) or (last_lw_spd_tgt != lw_spd_tgt):

            msg2 = 'rwheel_tgt_msg: {:.3f}; lwheel_tgt_msg: {:.3f};'.format(rw_spd_tgt,lw_spd_tgt)
            rospy.loginfo(msg2)
                       
            rwheel_spd_tgt_obj.publish(rwheel_tgt_msg)
            lwheel_spd_tgt_obj.publish(lwheel_tgt_msg)


        last_rw_spd_tgt = rw_spd_tgt
        last_lw_spd_tgt = lw_spd_tgt


        rdif = abs(last_rw_spd_tgt-rw_spd_tgt)
        ldif = abs(last_lw_spd_tgt-lw_spd_tgt)


        rate.sleep()

if __name__ == '__main__':
    try:
        twist2motor() #node_function must change
    except rospy.ROSInterruptException:
        pass