

#!/usr/bin/env python3

import time
import numpy as np

from sensor_msgs.msg import Image, PointCloud2

import cv2
from cv_bridge import CvBridge


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from turtlesim.msg import Pose

class SimpleDimple():

    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        # self.cmd_fl_wheel = rospy.Publisher('',  PointCloud2, queue_size=10)
        subscriber = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pcl_callback)

        self.rate = rospy.Rate(30)


    def pcl_callback(self, msg:PointCloud2):
        # rospy.loginfo("cmd vel command: {0} ".format(msg))

        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.w_z = msg.angular.z

    def spin(self):
        while not rospy.is_shutdown():
            # calculate the commands for wheels and publish
            # dsdsfsd
            self.rate.sleep()

    def shutdown(self):
        rospy.sleep(1)


simple = SimpleDimple()
simple.spin()


