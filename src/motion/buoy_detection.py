#! /usr/bin/python

import numpy as np
import cv2
import time
import rospy

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from motion_controller import *

class buoyDetector(object):

    def __init__(self, sim):

        self.targets = ["jia", "Vet"]

        self.targetVamp = self.targets[0]

        self.buoy_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", \
            BoundingBoxes, self.buoy_callback)

        self.controller = Controller(sim)

        if sim:
            self.cam_dim = [768, 492]
        else:
            self.cam_dim = [1280, 720]

        self.cam_area = self.cam_dim[0] * self.cam_dim[1]

        self.image_center = [self.cam_dim[0] / 2, self.cam_dim[1] / 2]

        self.error = [self.cam_dim[0] * 0.05, self.cam_dim[1] * 0.05]

        self.state = "findFirst"

        self.detected = False

        self.detected_idx = 0

        self.detected_box = None

        self.prev_detection = None

        self.execute = False

        self.centered = False

        self.firstHitAttitude = 0.0

        self.find_buoy_move = 0

    def buoy_callback(self, data):

        self.detected = False

        for i in range(len(data.bounding_boxes)):
            if data.bounding_boxes[i].Class == self.targetVamp:
                self.detected = True
                self.detected_idx = i
                self.bounding_box = data.bounding_boxes[i]

    def callExecute(self):

        rospy.loginfo("EXECUTING OBJECT DETECTOR")

        global targets

        while self.execute and not rospy.is_shutdown():

            if not self.detected:
                self.findBuoy()

            else:
                self.find_buoy_move = 0

                xmin = self.bounding_box.xmin
                xmax = self.bounding_box.xmax
                ymin = self.bounding_box.ymin
                ymax = self.bounding_box.ymax

                box_area = (xmax - xmin) * (ymax - ymin)

                self.ratio = float(ymax - ymin) / float(self.cam_dim[1])

                self.box_center = [(xmax + xmin) / 2, (ymax + ymin) / 2]

                self.targetFollower()

                rospy.loginfo("Detected, Box Area: " + str(box_area) + \
                    ", Ratio: " + str(self.ratio) + " Centered: " + str(self.centered))

                rospy.loginfo(self.bounding_box)

                if self.centered and (self.ratio > 0.25):

                    if self.state == "findFirst":

                        rospy.loginfo("HITTING AND GOING BACK")

                        self.firstHitAttitude = self.controller.attitude[2]

                        self.hitAndTurn()

                        self.state = "findSecond"
                        self.targetVamp = targets[1]

                    elif self.state == "findSecond":

                        rospy.loginfo("TRYING TO FIND SECOND")

                        vel = Twist()
                        vel.linear.x = 0.2
                        rospy.loginfo("Speed: " + str(vel.linear.x))
                        self.hitAndTurnHelper(vel, 4)

                        self.execute = False
                else:
                    vel = Twist()
                    vel.linear.x = 0.2
                    self.hitAndTurnHelper(vel, 1)
                self.detected = False

    def findBuoy(self):

        rospy.loginfo("Finding Buoy")

        vel = Twist()

        if self.find_buoy_move < 2:

            vel.linear.x = 0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move += 1

        elif self.find_buoy_move < 4:

            vel.linear.y = -0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move += 1

        elif self.find_buoy_move < 5:

            vel.linear.z = 0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move += 1

        elif self.find_buoy_move < 9:

            vel.linear.y = 0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move += 1

        elif self.find_buoy_move < 11:

            vel.linear.z = -0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move += 1

        elif self.find_buoy_move < 13:

            vel.linear.y = -0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move += 1

        elif self.find_buoy_move < 14:

            vel.linear.z = 0.2
            self.hitAndTurnHelper(vel, 1)

            self.find_buoy_move = 0

    def hitAndTurn(self):

        rospy.loginfo("GOING STRAIGHT")

        vel = Twist()
        vel.linear.x = 0.2
        rospy.loginfo("Speed: " + str(vel.linear.x))
        self.hitAndTurnHelper(vel, 4)

        rospy.loginfo("GOING BACK")

        vel = Twist()
        vel.linear.x = -0.2
        self.hitAndTurnHelper(vel, 5)

        rospy.loginfo("GOING UP")

        vel = Twist()
        vel.linear.z = 0.2
        self.hitAndTurnHelper(vel, 2)

        rospy.loginfo("GOING STRAIGHT")

        vel = Twist()
        vel.linear.x = 0.4
        self.hitAndTurnHelper(vel, 4)

        rospy.loginfo("ADJUSTING ATTITUDE")

        self.attitudeAdjuster()

        rospy.loginfo("GOING DOWN")

        vel = Twist()
        vel.linear.z = -0.2
        self.hitAndTurnHelper(vel, 2)


    def hitAndTurnHelper(self, vel, sec):

                start_time = time.time()

                # sleep rate

                while time.time() <= start_time + sec:
                    self.controller.pub_cmd_vel.publish(vel)
                    rospy.sleep(0.5)

                vel = Twist()
                self.controller.pub_cmd_vel.publish(vel)

                rospy.sleep(1)

    def attitudeAdjuster(self):

        required_orientation = self.firstHitAttitude + np.pi

        if required_orientation > 2 * np.pi:
            required_orientation -= 2 * np.pi

        vel = Twist()

        rospy.loginfo ("Current attitude: " + str(self.controller.attitude[2]))
        rospy.loginfo ("Required attitude: " + str(required_orientation))

        deg = self.controller.attitude[2] - required_orientation

        if deg < 0:
            deg += 2*np.pi

        if deg < np.pi:
            vel.angular.z = 0.1
        else:
            vel.angular.z = -0.1

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

    def targetFollower(self):

        vel = Twist()

        self.centered = False

        rospy.loginfo("Box Center: " + str(self.box_center) + "Image Center: " + str(self.image_center))

        if abs(self.box_center[1] - self.image_center[1]) > self.error[1]:
            if self.box_center[1] > self.image_center[1]:
                vel.linear.z = -0.1
            else:
                vel.linear.z = 0.1

            rospy.loginfo("yError")

            self.controller.pub_cmd_vel.publish(vel)
            rospy.sleep(1)

        elif abs(self.box_center[0] - self.image_center[0]) > self.error[0]:
            if self.box_center[0] > self.image_center[0]:
                vel.linear.y = -0.1
            else:
                vel.linear.y = 0.1

            rospy.loginfo("xError")

            self.controller.pub_cmd_vel.publish(vel)
            rospy.sleep(1)

        else:

            rospy.loginfo("CENTERED")
            self.centered = True
            rospy.sleep(0.2)

        vel = Twist()
        self.controller.pub_cmd_vel.publish(vel)

        rospy.sleep(0.1)

def main():
    rospy.init_node("buoy_detector")

    rospy.loginfo("Starting Buoy Detection")

    if len(sys.argv) > 1:
        if sys.argv[1] == "--sim":
            detect = buoyDetector(True)
    else:
        detect = buoyDetector(False)

    detect.execute = True

    detect.callExecute()

if __name__ == "__main__":
    main()
