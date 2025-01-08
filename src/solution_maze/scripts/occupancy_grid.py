#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

from utils import * 
from grid_map import *

# prob_to_log_odds, log_odds_to_prob, linear_mapping_of_values

# Constants
FLOOR_SIZE_X = 50  # meters
FLOOR_SIZE_Y = 50  # meters
RESOLUTION = 0.1  # meters per cell
WORLD_ORIGIN_X = -FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = -FLOOR_SIZE_Y / 2.0
ROBOT_ORIGIN_X = 0; ROBOT_ORIGIN_Y = 0; ROBOT_ORIGIN_THETA = 0;
MAP_SIZE_X = int(FLOOR_SIZE_X / RESOLUTION)
MAP_SIZE_Y = int(FLOOR_SIZE_Y / RESOLUTION)
PRIOR_PROB = 0.5
OCC_PROB = 0.8
FREE_PROB = 0.2

class Mapping:
    def __init__(self):
        rospy.init_node('mapping_node', anonymous=False)
        rospy.loginfo("creation of the map  has been started.")
        self.rate = rospy.Rate(12) # 3 Hz ///~ ~~~
        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # for scan msg
        self.from_frame = "camera_bottom_screw_frame" 
        self.to_frame = "map"

        self.ranges = []
        self.now = 0
        self.angle_min = 0
        self.angle_increment = 0
        self.robot_x = 0
        self.robot_y = 0
        
        rospy.Subscriber('/scan', LaserScan, self.get_scan, queue_size=10)
        # rospy.Subscriber('/odom', Odometry, self.get_odom, queue_size=3)  
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_pose, queue_size=1)  
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    
        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'map'
        self.occupancy_grid_msg.info = MapMetaData(
            width=MAP_SIZE_X,
            height=MAP_SIZE_Y,
            resolution=RESOLUTION,
            origin=Pose(
                position = Point(x=WORLD_ORIGIN_X, y=WORLD_ORIGIN_Y),
                orientation = Quaternion(x=0.0, y=0.0, z=0.0, w = 1.0)
            )
        )

        self.map = self.generate_map()
        self.pub_map()

    def generate_map(self):
        """
        Initialize map in the odom frame.
        """
        rospy.loginfo(f"Generating map of size: {(MAP_SIZE_X, MAP_SIZE_Y)}")
        # probability matrix in log-odds scale:
        return np.full(shape = (MAP_SIZE_X, MAP_SIZE_Y), fill_value = prob_to_log_odds(PRIOR_PROB))#-np.log(OCC_PROB)/np.log(FREE_PROB))

    def get_shape(self):
        """
		Get dimensions
		"""
        return np.shape(self.map)
    
    def mark_map(self, ix, iy, value):
        """
        Update a map's cell value.
        """
        try:
            self.map[ix, iy] += value
        except Exception as e:
            rospy.logerr(f"Problem when marking map: {e}")

    def get_map_val(self, ix, iy):
        """
        Get a map's cell value.
        """
        try:
            return self.map[ix, iy]
        except Exception as e:
            rospy.logerr(f"Problem when querying map: {e}")

    def pub_map(self):
        """
        Publish an OccupancyGrid to the /map topic.
        """
        map = log_odds_to_prob(self.map)
        map = linear_mapping_of_values(map)  # To visualize in Rviz

        self.occupancy_grid_msg.header.stamp = rospy.Time.now()
        # row-major order
        self.occupancy_grid_msg.data = map.astype(int).T.reshape(map.size, order='C').tolist()  
        self.map_publisher.publish(self.occupancy_grid_msg)

    def odom_coords_to_2d_array(self, x, y):
        """
        Transform a coord in the odom frame to coords in a 2D array.
        """
        ix = int(x / RESOLUTION) + MAP_SIZE_X // 2
        iy = int(y / RESOLUTION) + MAP_SIZE_Y // 2
        return ix, iy

    def get_perceptual_range(self, origin, target):
        """
        Get the grid cells that belong to the line between the origin and target.
        The returned points' coordinates are int-indexes of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
        """
        x0, y0 = self.odom_coords_to_2d_array(origin.point.x, origin.point.y)
        x1, y1 = self.odom_coords_to_2d_array(target.point.x, target.point.y)


        dx = np.abs(x1 - x0)
        dy = -np.abs(y1 - y0)

        sx = 1 if (x0 < x1) else -1
        sy = 1 if (y0 < y1) else -1

        err = dx + dy
        perceptual_range = []
        while True:
            perceptual_range.append((x0, y0))
            e2 = 2 * err
            if e2 >= dy:
                if x0 == x1: break
                err += dy
                x0 += sx
            if e2 <= dx:
                if y0 == y1: break
                err += dx
                y0 += sy
        return perceptual_range

    def get_odom(self, msg):
        # assert number > 0, f"number greater than 0 expected, got: {number}"
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y 
    
    def get_pose(self, msg):
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y 

    def get_scan(self, msg):
        """
        get a new LaserScan message
        """
        self.ranges = msg.ranges
        self.now = msg.header.stamp
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        
    def inverse_range_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        """
        if i == (len_perceptual_range - 1):
            return OCC_PROB
        else: # Assume a linear decrease in occupancy probability with distance
            return FREE_PROB
            # return FREE_PROB + (OCC_PROB - FREE_PROB) * (i / (len_perceptual_range - 1))

    def spin(self):
        rospy.sleep(5)
        while not rospy.is_shutdown():
            self.pub_map()

            transform_cam_to_odom = self.tf_buffer.lookup_transform("odom", self.from_frame, rospy.Time(0), rospy.Duration(1.0))
            transform_map_to_odom = self.tf_buffer.lookup_transform("odom", "map", rospy.Time(0), rospy.Duration(1.0))
            
            for i, range_value in enumerate(self.ranges[::40]):
                #filtering scan
                if np.isinf(range_value) or np.isnan(range_value):
                    continue
                angle = self.angle_min + 40 * i * self.angle_increment
                laser_point = PointStamped()
                laser_point.point.x = range_value * np.cos(angle)
                laser_point.point.y = range_value * np.sin(angle)
                laser_point.header.frame_id = self.from_frame
                laser_point.header.stamp = self.now

                base_point = PointStamped()
                base_point.header.frame_id = "map"#self.from_frame
                base_point.header.stamp = self.now
                base_point.point.x = self.robot_x
                base_point.point.y = self.robot_y
                # base_point.point.z = 0.35  

                laser_point_odom_frame = do_transform_point(laser_point, transform_cam_to_odom)
                base_point_odom = do_transform_point(base_point, transform_map_to_odom)
                perceptual_range = self.get_perceptual_range(
                    base_point_odom, laser_point_odom_frame
                )
                for j, (ix, iy) in enumerate(perceptual_range):
                    p = self.inverse_range_sensor_model(j, len(perceptual_range))
                    self.mark_map(ix, iy, value = prob_to_log_odds(p))

            
            self.rate.sleep()


try:
    map_node = Mapping()
    map_node.spin()
except rospy.ROSInterruptException:
    pass
