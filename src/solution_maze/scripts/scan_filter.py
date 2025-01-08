#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def filter_nan_ranges(ranges):
    """
    Replaces NaN values in the ranges array with the middle value of their neighbors.
    If multiple NaNs are in a row, they are replaced with the middle value of the non-NaN neighbors.
    """
    if not np.any(np.isnan(ranges)):
        return ranges

    filtered_ranges = ranges.copy()
    nan_indices = np.where(np.isnan(ranges))[0]

    for nan_idx in nan_indices:
        # Find left neighbor
        left_idx = nan_idx - 1
        while left_idx >= 0 and np.isnan(ranges[left_idx]):
            left_idx -= 1

        # Find right neighbor
        right_idx = nan_idx + 1
        while right_idx < len(ranges) and np.isnan(ranges[right_idx]):
            right_idx += 1

        if left_idx >= 0 and right_idx < len(ranges):
            middle_value = (ranges[left_idx] + ranges[right_idx]) / 2.0
        elif left_idx >= 0:
            middle_value = ranges[left_idx]
        elif right_idx < len(ranges):
            middle_value = ranges[right_idx]
        else:
            middle_value = 0  # This case should not occur if there is at least one valid range value

        filtered_ranges[nan_idx] = middle_value

    return filtered_ranges

def laser_scan_callback(scan_msg):
    filtered_ranges = filter_nan_ranges(np.array(scan_msg.ranges))

    filtered_scan_msg = LaserScan()
    filtered_scan_msg.header = scan_msg.header
    filtered_scan_msg.angle_min = scan_msg.angle_min
    filtered_scan_msg.angle_max = scan_msg.angle_max
    filtered_scan_msg.angle_increment = scan_msg.angle_increment
    filtered_scan_msg.time_increment =  100#scan_msg.time_increment
    filtered_scan_msg.scan_time = scan_msg.scan_time
    filtered_scan_msg.range_min = scan_msg.range_min
    filtered_scan_msg.range_max = scan_msg.range_max
    filtered_scan_msg.ranges = filtered_ranges
    filtered_scan_msg.intensities = scan_msg.intensities

    scan_pub.publish(filtered_scan_msg)

if __name__ == '__main__':
    rospy.init_node('laser_scan_nan_filter')

    scan_sub = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)
    scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)

    rospy.spin()
