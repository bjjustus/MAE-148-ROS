#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

def __init__(self):
        self.cam=cv2.VideoCapture(0)
        cv2.namedWindow("window", 1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, q$
def callback(self,data):
        hsv = cv2.cvtColor(self.cam, cv2.COLOR_BGR2HSV)
        lower=numpy.array((0, 50, 50))
        upper=numpy.array((50, 255, 255))
        mask=cv2.inRange(hsv, lower, upper)
        h,w,d= image.shape
        top=3*4/h
        bottom=h/4
        mask[0:bottom,0:w]=0  
	mask[top:h,0:w]=0		   
	M=cv2.moments(mask)
        if M>0:
           cx= int(M['m10']/M['m00'])
           cy= int(M['m01']/M['m00'])
           cv2.circle(self.cam, (cx, cy), 20, (0,0,255), -1)
           err=cx-w/2
           self.twist.linear.x=.1
           self.twist.angular.z=-float(err)/100

        cv2.imshow("window", self.cam)
        cv2.imshow("mask",mask)
        cv2.waitKey(0)
rospy.init_node('line_follower')
rospy.spin()

					   
					   
