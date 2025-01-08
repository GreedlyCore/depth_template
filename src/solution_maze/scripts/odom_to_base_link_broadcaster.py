#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose

class TF2Broadcaster:
    def __init__(self):
        rospy.init_node('tf2_odom_to_base_link')
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.last_time = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, msg):
        # Check if the timestamp is the same as the last one
        if self.last_time is not None and msg.header.stamp == self.last_time:
            return

        self.last_time = msg.header.stamp

        self.t.header.stamp = msg.header.stamp
        self.t.header.frame_id = msg.header.frame_id  # "odom"
        self.t.child_frame_id = msg.child_frame_id    # "base_link"

        self.t.transform.translation.x = msg.pose.pose.position.x
        self.t.transform.translation.y = msg.pose.pose.position.y
        self.t.transform.translation.z = msg.pose.pose.position.z
        self.t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.t)

if __name__ == '__main__':
    try:
        TF2Broadcaster()
    except rospy.ROSInterruptException:
        pass
