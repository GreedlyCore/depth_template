#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from math import hypot, atan2

def pcl_cb(msg):
    
    scan_msg = LaserScan()
    scan_msg.header = msg.header
    scan_msg.angle_min = -np.pi/2
    scan_msg.angle_max = np.pi/2 
    scan_msg.angle_increment = np.pi / 180 
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 1.0 / 30.0  # 30 Hz
    scan_msg.range_min = 0.2
    scan_msg.range_max = 10.0
    
    # Calculate number of beams needed
    num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
    scan_msg.ranges = [float('inf')] * num_readings
    
    # Convert PointCloud2 to list of (x,y,z) tuples
    gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    
    # Convert Cartesian to Polar coordinates
    for point in gen:
        x, y, z = point
        # Calculate distance and angle
        distance = np.hypot(x, y)
        angle = atan2(y, x)  # Returns angle between -pi to pi
        
        # Only consider points within scan boundaries
        if scan_msg.angle_min <= angle <= scan_msg.angle_max:
            # Find the appropriate index in the ranges array
            index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
            
            # Keep shortest distance at each angle
            if distance < scan_msg.ranges[index]:
                scan_msg.ranges[index] = distance
                
    # Replace inf with NaN if required by some laser scanners
    # scan_msg.ranges = [x if x != float('inf') else np.nan for x in scan_msg.ranges]
    
    scan_msg.intensities = []  # Empty intensities array
    scan_pub.publish(scan_msg)

if __name__ == '__main__':
    rospy.init_node('pcl_to_ls')
    scan_sub = rospy.Subscriber('/output', PointCloud2, pcl_cb)
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rospy.spin()