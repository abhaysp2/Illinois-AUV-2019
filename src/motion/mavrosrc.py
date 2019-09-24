#! /usr/bin/python

import rospy
import sys
import time

import numpy as np

from geometry_msgs.msg import *
from mavros_msgs.msg import *

class MavrosRC(object):

    def __init__(self):

        rospy.init_node("mavrosrc_node")

        self.rc_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        self.rc_sub = rospy.Subscriber("cmd_vel", Twist, self.rc_callback)

    def rc_callback(self, vel):

        # kPeriod = 0.1

        msg = OverrideRCIn()

        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = self.speedToPpm(vel.linear.z)
        msg.channels[3] = self.speedToPpm(vel.angular.z)
        msg.channels[4] = self.speedToPpm(vel.linear.x)
        msg.channels[5] = self.speedToPpm(vel.linear.y)

        msg.channels[6] = 1500

        self.rc_pub.publish(msg)

    def angleToPpm(self, angle):
        neg_M_PI = 1000
        M_PI = 2000
        ppm = (angle - (neg_M_PI)) / (M_PI - (neg_M_PI)) * (1000) + 1000

        return ppm

    def speedToPpm(self, speed):
        if speed > 1.0:
            speed = 1.0
        elif speed < -1.0:
            speed = -1.0

        return 1500 + speed * 500.0

def main():
    mavrc = MavrosRC()
    rospy.spin()

if __name__ == "__main__":
    main()
