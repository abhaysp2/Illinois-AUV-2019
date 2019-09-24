#! /usr/bin/python

from motion_controller import *
from buoy_detection import *

import rospy
import sys
import time

import numpy as np

required_orientation = 0.140
take_layout = ["turn", "head_to_gate", "gate", "path", "buoys"]

class FSM(object):

    def __init__(self, sim):
        self.sim = sim
        self.state = take_layout[0]
        self.controller = Controller(self.sim)
        self.goal_reached = False

        self.buoy_detector = buoyDetector(self.controller)

    def fsm_start(self):

        # Change Mode to Depth Hold Mode
        while (True and not self.sim):
            data = self.controller.changeToDepHold()
            if data == False:
                rospy.loginfo("Depth Hold Mode Failed, Retrying")
            else:
                break

        # ARM THE SUB IF NOT IN SIM
        while (True and not self.sim):
            data = self.controller.doArming()
            if data == False:
                rospy.loginfo("Arming Failed, Retrying")
            else:
                break

        # Turn REQUIRED amount to required_orientation
        # self.Turn()

        # DIVE REQUIRED AMOUNT BASED ON SECONDS
        # ADD Support for dive by height using Pressure Sensor
        self.Dive(4)

        rospy.sleep(1)

        self.Turn(0)

        rospy.sleep(1)

        self.goStraight(10)

        rospy.sleep(1)

        self.Turn(7.5)

        rospy.sleep(3)

        self.Turn(0)

        rospy.sleep(1)

        self.goStraight(10)

        self.Dive(4)

        rospy.sleep(1)

        self.goStraight(10)

        # TESTING

        # self.Dive(2)
        #
        # rospy.sleep(1)
        #
        # self.Turn(0)
        #
        # rospy.sleep(1)
        #
        # self.goStraight(2)
        #
        # self.Turn(5)
        #
        # rospy.sleep(3)
        #
        # self.Turn(0)

        # START Darknet Execution

    # TRY SETPOINT_ATTITUDE
    def Turn_helper(self):

        global required_orientation

        vel = Twist()

        rospy.loginfo ("Current attitude: " + str(self.controller.attitude[2]))
        rospy.loginfo ("Required attitude: " + str(required_orientation))

        deg = self.controller.attitude[2] - required_orientation

        if deg < 0:
            deg += 2*np.pi

        if deg < np.pi:
            vel.angular.z = 0.125
        else:
            vel.angular.z = -0.125

        rospy.loginfo("STARTING TURNING")

        while abs(deg) > np.pi/20 and not rospy.is_shutdown():

            deg = abs(self.controller.attitude[2] - required_orientation)

            # rospy.loginfo(self.controller.attitude[2])

            if deg > np.pi:
                deg -= 2*np.pi
            # rospy.loginfo(str(self.controller.attitude))
            self.controller.pub_cmd_vel.publish(vel)
            rospy.sleep(1)

        vel.angular.z = 0.0
        self.controller.pub_cmd_vel.publish(vel)
        rospy.loginfo("TURNING DONE")

        rospy.sleep(1)

    def Turn(self, sec):

        if sec == 0:
            self.Turn_helper()
            return

        vel = Twist()

        rospy.loginfo("Turn 2-3 Times")
        rospy.loginfo ("Current attitude: " + str(self.controller.attitude[2]))
        rospy.loginfo ("Required attitude: " + str(required_orientation))

        vel.angular.z = 0.4

        rospy.loginfo("STARTING TURNING")

        start_time = time.time()

        while time.time() < start_time + sec:
            self.controller.pub_cmd_vel.publish(vel)
            rospy.sleep(1)

        vel.angular.z = 0.0
        self.controller.pub_cmd_vel.publish(vel)
        rospy.sleep(1)

        rospy.loginfo("TURNING DONE")

    def Dive(self, sec):
        # Add distance or seconds

        rospy.loginfo("DIVING")

        vel = Twist()
        vel.linear.z = -0.5

        rospy.loginfo("Speed: " + str(vel.linear.z))
        start_time = time.time()

        # sleep rate

        while time.time() <= start_time + sec:
            self.controller.pub_cmd_vel.publish(vel)
            rospy.sleep(0.5)

        vel = Twist()
        vel.angular.z = 0.0
        self.controller.pub_cmd_vel.publish(vel)
        rospy.sleep(1)
        rospy.loginfo("DIVING OVER")

    def goStraight(self, sec):

        rospy.loginfo("GOING STRAIGHT")

        vel = Twist()

        vel.linear.x = 0.4

        rospy.loginfo("Speed: " + str(vel.linear.x))
        start_time = time.time()

        # sleep rate

        while time.time() <= start_time + sec:
            self.controller.pub_cmd_vel.publish(vel)
            rospy.sleep(0.5)

        vel = Twist()
        vel.linear.x = 0.0
        self.controller.pub_cmd_vel.publish(vel)

        rospy.sleep(1)
        rospy.loginfo("GO STRAIGHT OVER")


def main():

    rospy.init_node("fsm")

    if len(sys.argv) > 1:
        if sys.argv[1] == "--sim":
            fsm = FSM(True)
    else:
        fsm = FSM(False)

    fsm.fsm_start()

if __name__ == "__main__":
    main()
