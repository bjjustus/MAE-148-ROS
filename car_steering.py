#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist

def __init__(self):
    self.speed=rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, callback)    

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    return math.atan(wheelbase / radius)

def callback(data):
   rospy.loginfo(data)

while not rospy.is_shutdown():
    rospy.init_node('car')
    wheelbase = 1
    neutral = 370
    PWM_max = 460
    PWM_min = 290
    k = 10
    msg=Twist()
    sub=rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, callback)
    x=msg.linear.x
    z=msg.linear.z
    steering = convert_trans_rot_vel_to_steering_angle(x, z, wheelbase)
    PWM_steering = neutral + k * steering
    print(x)
    if PWM_steering > PWM_max:
        PWM_steering = PWM_max
    elif PWM_steering < PWM_min:
        PWM_steering = PWM_min

 
