#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

from utils import * 
from transformations import *

class TF2Broadcaster:
    def __init__(self):
        rospy.init_node('tf2_odom_to_baselink')
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        # Store the last timestamp to avoid redundant data
        self.last_time = None
        self.zero_quat = Quaternion(x=0.0, y=0.0, z=0.0, w = 1.0)
        # Subscribe to the /odom topic
        rospy.Subscriber('/ground/state', Odometry, self.world_callback, queue_size=1)  
        rospy.spin()
 
    def world_callback(self, msg):
        # Check if the timestamp is the same as the last one
        if self.last_time is not None and msg.header.stamp == self.last_time:
            return
        # Update the last timestamp
        self.last_time = msg.header.stamp
        # Use the timestamp from the odometry message
        self.t.header.stamp = msg.header.stamp
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_link"
        # Set translation
        self.t.transform.translation.x = 0.0
        self.t.transform.translation.y = 0.0
        self.t.transform.translation.z = 0.0
        # Set rotation
        self.t.transform.rotation = self.zero_quat
        self.br.sendTransform(self.t)

if __name__ == '__main__':
    TF2Broadcaster()
    
