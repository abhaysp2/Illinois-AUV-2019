#! /usr/bin/python

import numpy as np
import rospy

from motion_controller import *

def main():
    controller = Controller(False)

    controller.changeToDepHold()

    controller.doArming()

    while not rospy.is_shutdown():

        rospy.init_node("attitudeMeasurer")
        rospy.loginfo("Yaw Measurer")
        rospy.loginfo(controller.attitude[2])
        rospy.sleep(1)

if __name__ == "__main__":
    main()
