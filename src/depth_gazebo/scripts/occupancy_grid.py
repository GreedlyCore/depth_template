#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point, do_transform_pose

from utils import * 
from grid_map import *
from transformations import *

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
        rospy.loginfo("creation of the map -- started.")
        self.rate = rospy.Rate(30) 
        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # for scan msg
        self.from_frame = "lidar_link" #"camera_bottom_screw_frame" 
        self.to_frame = "map"

        self.ranges = []
        self.angle_min = 0
        self.range_max = 0
        self.angle_increment = 0
        self.robot_x = 0
        self.robot_y = 0
        self.robot_pixel_x = 0
        self.robot_pixel_y = 0

        self.z_random = 0.05
        self.laser_z_hit = 0.95
        self.laser_sigma_hit = 0.2

        self.l0 = np.round(np.log(OCC_PROB/FREE_PROB),3)#prob_to_log_odds(PRIOR_PROB)
        self.OCC_L = np.round(1.5 * self.l0, 3)
        self.FREE_L = np.round(0.5 * self.l0, 3)
        self.alpha = 0.05 # already divided by two
        self.beta = 0.3 # angle in rads (???) # the opening angle of this sensor is

        rospy.Subscriber('/scan_filtered', LaserScan, self.get_scan, queue_size=1)
        rospy.Subscriber('/ground/state', Odometry, self.get_gazebo, queue_size=1)  
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        test_scan = rospy.wait_for_message('/scan', LaserScan, rospy.Duration(5.0))
        self.angles = np.zeros(len(test_scan.ranges)); self.angles.astype(np.float64)
        self.angle_min = test_scan.angle_min
        self.angle_max = test_scan.angle_max
        self.range_max = test_scan.range_max

        self.angles = np.linspace(self.angle_min, self.angle_max, len(test_scan.ranges))

        print(f"INVERSE_MODEL_CONFIG:\nl0: {self.l0}  \nl_occ: {self.OCC_L} \nl_free: {self.FREE_L} \nalpha/2 coeff: {self.alpha} \nbeta/2 coeff: {self.beta}")

        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'world'
        self.occupancy_grid_msg.info = MapMetaData(
            width=MAP_SIZE_X,
            height=MAP_SIZE_Y,
            resolution=RESOLUTION,
            origin=Pose(
                position = Point(x=WORLD_ORIGIN_X, y=WORLD_ORIGIN_Y),
                orientation = Quaternion(x=0.0, y=0.0, z=0.0, w = 0.0)
            )
        )

        self.map = self.generate_map(0.0)
        self.likelihood_field = self.generate_map(0.0)
        self.pub_map()
        rospy.loginfo("creation of the map -- finished.")

    def generate_map(self, value):
        """
        Initialize map in the odom frame.
        """
        rospy.loginfo(f"Generating map of size: {(MAP_SIZE_X, MAP_SIZE_Y)}")
        # probability matrix in log-odds scale:
        #-np.log(OCC_PROB)/np.log(FREE_PROB) 
        # TODO is minus needed? really?
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

        self.occupancy_grid_msg.header.stamp = rospy.Time.now()
        # row-major order
        # map = self.likelihood_field * 100
        self.occupancy_grid_msg.data = self.map.astype(int).T.reshape(self.map.size, order='C').tolist()  
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
        x1, y1 = self.coords_to_2d_array(robot_pose[0] + obs[0] * np.cos(robot_pose[2] + obs[1]), robot_pose[1] + obs[0] * np.sin(robot_pose[2] + obs[1]))


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
                rospy.loginfo("info message")
            
            q =  self.laser_z_hit * np.exp(-(dist ** 2) / (2 * self.laser_sigma_hit ** 2)) + self.z_random/self.range_max
            self.likelihood_field[x, y] = q



    
    def inverse_range_sensor_model(self,x, y, robot_pose, ranges):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        x = x_i
        y = y_i
        alpha = ALPHA/2
        beta = BETA/2
        """
        robot_pixel_x, robot_pixel_y = self.coords_to_2d_array(robot_pose[0],robot_pose[1])
        # * RESOLUTION --> [m] ???
        r = np.sqrt( (x - robot_pixel_x) **2 + (y - robot_pixel_y)**2 ) * RESOLUTION
        phi = angle_diff(np.arctan2((y - robot_pixel_y), (x - robot_pixel_x))  ,   robot_pose[2])
        k = np.argmin(np.absolute(np.array([angle_diff(phi, angle) for angle in self.angles]))) 
        
        if not np.isnan(ranges[k]):
            if r > min(self.range_max, ranges[k] + self.alpha) or abs( angle_diff( phi, self.angles[k]))> self.beta:
                return self.l0 # out of range case
            if ranges[k] < self.range_max and abs(r - ranges[k]) < self.alpha:
                return self.OCC_L # if range for cell is over  ( +-self.alpha ) than z^k_t - our measurement
            if r <= ranges[k]:
                return self.FREE_L # else case
    
    def spin(self):
        rospy.sleep(2) # give an extra time to init everything
        while not rospy.is_shutdown():
            self.likelihood_model([self.robot_x, self.robot_y, self.robot_theta], self.ranges)
            for i in range(MAP_SIZE_X):
                for j in range(MAP_SIZE_Y):
                    if self.likelihood_field[i, j] >= 0.3: # occ case
                        self.map[i, j] = 100
                    else: self.map[i, j] = 0
            self.pub_map()
            self.rate.sleep()
try:
    map_node = Mapping()
    map_node.spin()
except rospy.ROSInterruptException:
    pass
