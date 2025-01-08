#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose
from transformations import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.linalg import inv

class TF2Broadcaster:
    def __init__(self):
        rospy.init_node('tf2_start')
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.last_time = None
        self.first_time = True
        self.odom = None
        self.pose = None

        # Retrieve the initial pose from the parameter server

        
        self.init_x = None
        self.init_y = None
        self.init_z = None

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, msg):
            # Check if the timestamp is the same as the last one
            if self.last_time is not None and msg.header.stamp == self.last_time:
                return
            # if self.first_time:
            init_pose = rospy.get_param('/init_pose', {'x': 0.0, 'y': 0.0, 'z': 0.0})
            self.init_x = init_pose['x']
            self.init_y = init_pose['y']
            self.init_z = init_pose['z']
                # self.first_time = False
                
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            current_z = msg.pose.pose.position.z

            relative_translation = np.array([current_x - self.init_x, current_y - self.init_y, current_z - self.init_z])
            self.last_time = msg.header.stamp
            self.t.header.stamp = msg.header.stamp
            # self.t.header.stamp = rospy.Time.now()
            self.t.header.frame_id = "start"
            self.t.child_frame_id = "base_link" # odom

            self.t.transform.translation.x = relative_translation[0]
            self.t.transform.translation.y = relative_translation[1]
            self.t.transform.translation.z = relative_translation[2]
            # copy rotation
            self.t.transform.rotation = msg.pose.pose.orientation

            # Publish the transform
            self.br.sendTransform(self.t)

if __name__ == '__main__':
    try:
        TF2Broadcaster()
    except rospy.ROSInterruptException:
        pass
