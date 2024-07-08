#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class TF2Broadcaster:
    def __init__(self):
        rospy.init_node('tf2_world_to_base_link')

        # Initialize the transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        
        # Store the last timestamp to avoid redundant data
        self.last_time = None

        # Subscribe to the /odom topic
        rospy.Subscriber('/ground/state', Odometry, self.odom_callback)

        rospy.spin()

    def odom_callback(self, msg):
        # Check if the timestamp is the same as the last one
        if self.last_time is not None and msg.header.stamp == self.last_time:
            return

        # Update the last timestamp
        self.last_time = msg.header.stamp

        # Use the timestamp from the odometry message
        self.t.header.stamp = msg.header.stamp
        self.t.header.frame_id = msg.header.frame_id  # "odom"
        self.t.child_frame_id = msg.child_frame_id    # "base_link"
        
        # Set translation
        self.t.transform.translation.x = msg.pose.pose.position.x
        self.t.transform.translation.y = msg.pose.pose.position.y
        self.t.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation
        self.t.transform.rotation = msg.pose.pose.orientation

        # Send the transform
        self.br.sendTransform(self.t)

if __name__ == '__main__':
    try:
        TF2Broadcaster()
    except rospy.ROSInterruptException:
        pass
