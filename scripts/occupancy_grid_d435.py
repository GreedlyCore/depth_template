#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

from utils import * 
from grid_map import *
from transformations import *
import sys
# from dynamic_reconfigure.server import Server


# Constants
FLOOR_SIZE_X = 50  # meters
FLOOR_SIZE_Y = 50  # meters
RESOLUTION = 0.1  # meters per cell
WORLD_ORIGIN_X = -FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = -FLOOR_SIZE_Y / 2.0
ROBOT_ORIGIN_X = 0; ROBOT_ORIGIN_Y = 0; ROBOT_ORIGIN_THETA = 0;
MAP_SIZE_X = int(FLOOR_SIZE_X / RESOLUTION)
MAP_SIZE_Y = int(FLOOR_SIZE_Y / RESOLUTION)
PRIOR_PROB = 0.6
OCC_PROB = 0.8
FREE_PROB = 0.3

class Mapping:
    def __init__(self):
        rospy.init_node('mapping_node', anonymous=False)
        rospy.loginfo(" --- naive mapping started --- ")
        self.rate = rospy.Rate(30) 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.from_frame = "camera_link" 
        self.to_frame = "map"

        self.ranges = []
        self.angle_min = 0
        self.range_max = 0
        self.angle_increment = 0.0
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0.0
        self.robot_pixel_x = 0
        self.robot_pixel_y = 0

        self.z_random = 0.05 # default value from amcl setup
        self.laser_z_hit = 0.95 # default value from amcl setup
        self.laser_sigma_hit = 0.2 # default value from amcl setup

        self.l0 = np.round(np.log(OCC_PROB/FREE_PROB),3)#prob_to_log_odds(PRIOR_PROB)
        self.OCC_L = np.round(1.5 * self.l0, 3)
        self.FREE_L = np.round(0.5 * self.l0, 3)
        
        # self.srv = Server(mapping, self.reconfigure_callback)
        self.alpha = rospy.get_param("/mapping_node/alpha")  # Default value
        self.beta = rospy.get_param("/mapping_node/beta")     # Default value
        # Take laser beam distances after filtering
        rospy.Subscriber('/scan_filtered', LaserScan, self.get_scan, queue_size=1)
        # Take localization from gazebo
        rospy.Subscriber('/ground/state', Odometry, self.get_gazebo, queue_size=1)  
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=5)
        
        # Waiting for first lidar messages
        test_scan = rospy.wait_for_message('/scan', LaserScan, rospy.Duration(5.0))
        self.angles = np.zeros(len(test_scan.ranges)); self.angles.astype(np.float64)
        self.angle_min = test_scan.angle_min
        self.angle_max = test_scan.angle_max
        self.angle_increment = test_scan.angle_increment
        self.range_max = test_scan.range_max
        self.angles = np.linspace(self.angle_min, self.angle_max, len(test_scan.ranges))
        
        self.occupancy_grid_msg = OccupancyGrid()
        # world frame starts from original gazebo (0,0,0) point
        self.occupancy_grid_msg.header.frame_id = 'world'
        self.occupancy_grid_msg.info = MapMetaData(
            width=MAP_SIZE_X,
            height=MAP_SIZE_Y,
            resolution=RESOLUTION,
            # The origin of the map [m, m, rad].  
            # This is the real-world pose of the
            # cell (0,0) in the map
            origin=Pose(
                position = Point(x=WORLD_ORIGIN_X, y=WORLD_ORIGIN_Y),
                orientation = Quaternion(x=0.0, y=0.0, z=0.0, w = 1.0)
            )
        )
        # fill whole map with no occupancy value
        self.map = self.generate_map(0.0)
        self.likelihood_field = self.generate_map(0.0)
        self.pub_map()
        rospy.loginfo(" --- mapping initted --- ")

    # def reconfigure_callback(self, config, level):
    #     self.alpha = rospy.get_param("/alpha")
    #     self.beta = rospy.get_param("/alpha")
    #     rospy.loginfo(f"Reconfigure Request: alpha: {self.alpha}, beta: {self.beta}")
    #     return config

    def generate_map(self, value):
        """
        Initialize map in the odom frame.
        """
        rospy.loginfo(f"Generating map of size: {(MAP_SIZE_X, MAP_SIZE_Y)}")
        return np.full(shape = (MAP_SIZE_X, MAP_SIZE_Y), fill_value = value)

    def get_shape(self):
        """
		Get dimensions
		"""
        return np.shape(self.map)
    
    def mark_map(self, ix, iy, value):
        """
        Update a map's cell value with LOG.
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

    def coords_to_2d_array(self, x, y):
        """
        Transform a coord in the WORLD frame to coords in a 2D grid array.
        """
        ix = int((x - WORLD_ORIGIN_X) / RESOLUTION)
        iy = int((y - WORLD_ORIGIN_Y) / RESOLUTION)
        return ix, iy

    def get_perceptual_range(self, robot_pose, obs):
        """
        Get the grid cells that belong to the line between the origin and target.
        The returned points' coordinates are int-indexes of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
        """
        x0, y0 = self.coords_to_2d_array(robot_pose[0], robot_pose[1])
        x1, y1 = self.coords_to_2d_array(obs[0], obs[1])            

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
    
    # in earth frame, but --> map
    def get_gazebo(self, msg):
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y 
        self.robot_theta =  transform_orientation(msg.pose.pose.orientation)
            
    def get_scan(self, msg):
        """
        get a new LaserScan message
        """
        self.ranges = msg.ranges

    def likelihood_model(self, robot_pose, ranges):
        arr = []
        
        for i in range(MAP_SIZE_X):
            for j in range(MAP_SIZE_Y):
                if self.map[i , j] == 100:
                    arr.append((i, j))
        # print(f"NOW arr is: lrn - {len(arr)}\n -- {arr}")
        for k in range(len(ranges)):
            if ranges[k] > self.range_max: continue
            dist = 1000
            #laser point
            x, y = self.coords_to_2d_array(robot_pose[0] + ranges[k] * np.cos(robot_pose[2] + self.angles[k]), 
                                             robot_pose[1] + ranges[k] * np.sin(robot_pose[2] + self.angles[k]))
            
            for (occ_x,occ_y) in arr:
                r = np.sqrt( (x - occ_x)**2 + (y - occ_y)**2  ) * RESOLUTION
                if r < dist: dist = r
            if dist == 1000: 
                dist = 0
                # rospy.loginfo("info message")
            
            q =  self.laser_z_hit * np.exp(-(dist ** 2) / (2 * self.laser_sigma_hit ** 2)) + self.z_random/self.range_max
            self.likelihood_field[x, y] = q



    def inverse_range_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        """
        if i == (len_perceptual_range - 1): return OCC_PROB
        else: return FREE_PROB

    def spin(self):
            angle = np.linspace(self.angle_min, self.angle_max, len(self.ranges)) 
            while not rospy.is_shutdown():      
                base_point_world = [self.robot_x, self.robot_y, self.robot_theta]
                ranges = self.ranges
                if len(ranges) != 0:
                    for i in range(len(ranges)):
                        # lidar_to_base !! TF make static
                        proj = [ ranges[i] * np.cos(base_point_world[2] + angle[i]), ranges[i] * np.sin(base_point_world[2] + angle[i])]
                        laser_point_world = [base_point_world[0] + proj[0], base_point_world[1] + proj[1] ]
                        perceptual_range = self.get_perceptual_range(
                            base_point_world, laser_point_world
                        )
                        for j, (ix, iy) in enumerate(perceptual_range):
                            p = self.inverse_range_sensor_model(j, len(perceptual_range))
                            self.mark_map(ix, iy, value = prob_to_log_odds(p))
                    
                    self.pub_map()
                    self.rate.sleep()
try:
    map_node = Mapping()
    map_node.spin()
except rospy.ROSInterruptException:
    pass