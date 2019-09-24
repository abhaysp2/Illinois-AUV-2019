#! /usr/bin/python

import numpy as np
import rospy

import tf

from geometry_msgs.msg import *
from mavros_msgs.msg import State
from mavros_msgs.srv import *
from nav_msgs.msg import Odometry

class Controller(object):

    def __init__(self, sim):

        if sim:
            odom_topic = "/rexrov/pose_gt"
            vel_topic = "/rexrov/cmd_vel"
            accel_topic = "/rexrov/cmd_accel"
        else:
            odom_topic = "/mavros/global_position/local"
            vel_topic = "/cmd_vel"
            accel_topic = "/mavros/setpoint_accel/accel"

        # rospy.loginfo(vel_topic)

        # SUBCRIBERS
        if not sim:
            rospy.Subscriber("/mavros/state", State, self.state_callback)

        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # ADD CAMERA SUBS


        # PUBLISHERS
        self.pub_cmd_vel = rospy.Publisher(vel_topic, Twist, queue_size=10)

        self.pub_accel = rospy.Publisher(accel_topic, Accel, queue_size=10)

        # if not sim:
        #     self.pub_attitude = rospy.Publisher("/mavros/setpoint_raw/attitude")

        # SERVICES

        if not sim:
            self.arming_agent = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # VARIABLES
        self.mavros_state = State()
        self.odom = Odometry()
        self.attitude = [0,0,0]
        self.vel = Twist()
        self.arm_state = False

    # FUNCTION TO ARM THE SUB
    def doArming(self):
        data = self.arming_agent(True)
        rospy.loginfo("Arming Callback")
        rospy.loginfo(data)
        return data

    # FUNCTION TO DISARM THE SUB
    def doDisarm(self):
        data = self.arming_agent(False)
        rospy.loginfo("Disarm Callback")
        rospy.loginfo(data)

    # Changes mode to Depth Hold
    def changeToDepHold(self):
        rospy.loginfo("Depth Hold Callback")
        data = self.set_mode(custom_mode="ALT_HOLD")
        rospy.loginfo(data)
        return data

    # Odom Callback
    def odom_callback(self, data):
        self.odom = data
        orien = data.pose.pose.orientation

        quart = [orien.x, orien.y, orien.z, orien.w]
        self.attitude = tf.transformations.euler_from_quaternion(quart)
        self.attitude = list(self.attitude)

        if self.attitude[2] < 0:
            self.attitude[2] += 2*np.pi

        self.vel = data.twist.twist

    def state_callback(self, data):
        self.mavros_state = data

        if not self.mavros_state.armed:
            self.doArming()

        if self.mavros_state.mode != "ALT_HOLD":
            self.changeToDepHold()
        # ADD logic depending on mavros states

    def velocity_publisher(self, data):
        self.pub_cmd_vel.publish(data)

    def pubAttitudeData(self):
        rospy.loginfo("YAW MEASURER")
        rospy.loginfo("Current Yaw: " + str(self.attitude[2]))
