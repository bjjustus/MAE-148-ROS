#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit
import adafruit_motor.servo


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    return math.atan(wheelbase / radius)

def callback(msg):
   rospy.loginfo(msg)

def listener():
    rospy.init_node('car')
    wheelbase = 1
    neutral = 370
    PWM_max = 460
    PWM_min = 290
    msg=Twist()
    rospy.Subscriber('/cmd_vel', Twist, callback)
    x=msg.linear.x
    z=msg.linear.z
    steering = convert_trans_rot_vel_to_steering_angle(x, z, wheelbase)
    print(x)

    kit = ServoKit(channels=16)
    servo = adafruit_motor.servo.Servo(servo_channel)
    kit.servo[1].set_pulse_width_range(290, 460)
    kit.servo[1].angle = 57.2958 * steering
    kit.continuous_servo[0].throttle = linear.x
    
    rospy.spin()
    
    
if __name__ == '__main__':
     listener()

