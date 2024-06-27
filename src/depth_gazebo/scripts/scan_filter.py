#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def scan_callback(scan):
    filtered_scan = LaserScan()
    
    filtered_scan.header = scan.header
    filtered_scan.angle_min = scan.angle_min
    filtered_scan.angle_max = scan.angle_max
    filtered_scan.angle_increment = scan.angle_increment
    filtered_scan.time_increment = scan.time_increment
    filtered_scan.scan_time = scan.scan_time
    filtered_scan.range_min = scan.range_min
    filtered_scan.range_max = scan.range_max
    
    # Filter out NaNs and replace with +inf
    filtered_ranges = [r if not np.isnan(r) else float('inf') for r in scan.ranges]
    filtered_scan.ranges = filtered_ranges
    
    filtered_scan.intensities = scan.intensities
    
    scan_pub.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node('scan_filter_node')
    
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
    
    rospy.spin()
