#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class TF2Broadcaster:
    def __init__(self):
        rospy.init_node('tf2_odom_to_base_link')

        # Initialize the transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()

        # Subscribe to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.spin()

    def odom_callback(self, msg):
        self.t.header.stamp = rospy.Time.now()
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
