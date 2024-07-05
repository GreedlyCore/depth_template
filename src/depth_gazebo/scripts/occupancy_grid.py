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
from trans import *
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
PRIOR_PROB = 0.6
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
        self.range_max = 0
        self.angle_increment = 0
        self.robot_x = 0
        self.robot_y = 0
        self.robot_pixel_x = 0
        self.robot_pixel_y = 0

        self.l0 = np.round(np.log(OCC_PROB/FREE_PROB),3)#prob_to_log_odds(PRIOR_PROB)
        self.OCC_L = np.round(1.5 * self.l0, 3)
        self.FREE_L = np.round(0.5 * self.l0, 3)
        self.alpha = 0.1
        # the opening angle of this sensor is, already divided by two
        self.beta = 0.003 # angle in rads

        rospy.Subscriber('/scan', LaserScan, self.get_scan, queue_size=10)
        # rospy.Subscriber('/odom', Odometry, self.get_odom, queue_size=3)  
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_pose, queue_size=1)  
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        test_scan = rospy.wait_for_message('/scan', LaserScan, rospy.Duration(5.0))
        self.angles = np.zeros(len(test_scan.ranges[::10])); self.angles.astype(np.float64)
        self.angle_min = np.round(test_scan.angle_min, 4)
        self.range_max = test_scan.range_max
        self.angle_increment = np.round(test_scan.angle_increment, 4)

        for i in range(len(test_scan.ranges[::10])):
            self.angles[i] = self.angle_min + (10 * i * self.angle_increment)

        print(f"INVERSE_MODEL_CONFIG:\nl0: {self.l0}  \nl_occ: {self.OCC_L} \nl_free: {self.FREE_L} \nalpha/2 coeff: {self.alpha} \nbeta/2 coeff: {self.beta}")

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
        #-np.log(OCC_PROB)/np.log(FREE_PROB) 
        # TODO is minus needed? really?
        return np.full(shape = (MAP_SIZE_X, MAP_SIZE_Y), fill_value = 0.0)

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
        Transform a coord in the MAP frame to coords in a 2D grid array.
        """
        # int
        ix = ((x - WORLD_ORIGIN_X) / RESOLUTION)
        iy = ((y - WORLD_ORIGIN_Y) / RESOLUTION)
        row = min(max(int(iy), 0), MAP_SIZE_Y)
        col = min(max(int(ix), 0), MAP_SIZE_X)
        return row, col #ix, iy

    def get_perceptual_range(self, origin, target):
        """
        Get the grid cells that belong to the line between the origin and target.
        The returned points' coordinates are int-indexes of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
        """
        x0, y0 = self.coords_to_2d_array(origin[0], origin[1])
        x1, y1 = self.coords_to_2d_array(target[0], target[0])


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
        pass
        # self.robot_x =  msg.pose.pose.position.x 
        # self.robot_y =  msg.pose.pose.position.y 
        # self.robot_theta = np.round(transform_orientation(msg.pose.pose.orientation), 4)

    def get_pose(self, msg):
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y
        self.robot_theta = np.round(transform_orientation(msg.pose.pose.orientation), 4)
        assert self.robot_x != 0 and self.robot_y != 0, f"robot pose is zero"

    def get_scan(self, msg):
        """
        get a new LaserScan message
        """
        self.ranges = msg.ranges
        self.now = msg.header.stamp

        
    def inverse_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        """
        if i == (len_perceptual_range - 1):
            return OCC_PROB
        else: # Assume a linear decrease in occupancy probability with distance
            return FREE_PROB
            # return FREE_PROB + (OCC_PROB - FREE_PROB) * (i / (len_perceptual_range - 1))

    def inverse_range_sensor_model(self,x, y):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        x = x_i
        y = y_i
        alpha = ALPHA/2
        beta = BETA/2
        """
        self.robot_pixel_x, self.robot_pixel_y = self.coords_to_2d_array(self.robot_x,self.robot_y)
        # * RESOLUTION --> [m] ???
        r = np.round(np.sqrt( (x - self.robot_pixel_x)**2 + (y - self.robot_pixel_y)**2 ) * RESOLUTION , 4)
        phi = np.arctan2((y - self.robot_pixel_y), (x - self.robot_pixel_x)) - self.robot_theta
        # Normalize phi to be within [-π, π]
        phi = np.round( (phi + np.pi) % (2 * np.pi) - np.pi , 4)
        
        # k = np.argmin( np.absolute(phi - self.angles) )
        # print(f"ROBOT THETA: {self.robot_theta}")
        # print(f"ANGLES: r = {r} phi = {phi} k = {k} \n POSES: ({x}, {y}) , ({self.robot_pixel_x}, {self.robot_pixel_y})\n")
        # assert r > 0 and phi != 0, f"(r, phi) == (negative , 0)"
        
        
        # print(f"{r} > {min(self.range_max, np.round(self.ranges[k], 4) + self.alpha)} or {np.round(abs(phi - self.angles[k]), 5)} > {self.beta}")
        for k in range(len(self.angles)):
            if r > min(self.range_max, np.round(self.ranges[k], 4) + self.alpha) or np.round(abs(phi - self.angles[k]), 5) > self.beta:
                return self.l0
            if self.ranges[k] < self.range_max and abs(r - self.ranges[k]) < self.alpha:
                return self.OCC_L
            if r <= self.ranges[k]:
                return self.FREE_L
            else:
                raise("Sorry, no")


    def spin(self):
        rospy.sleep(5)
        while not rospy.is_shutdown():
            self.pub_map()
            # transform_map_to_odom = self.tf_buffer.lookup_transform("odom", "map", rospy.Time(0), rospy.Duration(1.0))
        
            for i, range_value in enumerate(self.ranges[::10]):
                #filtering scan
                if np.isinf(range_value) or np.isnan(range_value):
                    continue

                # laser_point - one dot from laserscan in camera_frame
                # base_point - estimated robot position via /amcl_pose
                # provide projections --> we need 4x1 vector because we have 4x4 matrix
                laser_point = np.array([range_value * np.cos(self.angles[i]), range_value * np.sin(self.angles[i]), 1, 1])
                base_point = np.array([self.robot_x, self.robot_y])

                transform_cam_to_map = msg_to_se3( self.tf_buffer.lookup_transform("map", self.from_frame, rospy.Time(0), rospy.Duration(1.0)))
                laser_point_odom_frame = transform_point(laser_point, transform_cam_to_map)
                # base_point_odom = transform_point(base_point, transform_map_to_odom)

                perceptual_range = self.get_perceptual_range(
                    base_point, laser_point_odom_frame)
                for (ix, iy) in perceptual_range:
                    # p = self.inverse_sensor_model(j, len(perceptual_range))
                    L = self.inverse_range_sensor_model(ix, iy) - self.l0
                    # if L > 0: print(f"L is: {L}")
                    self.mark_map(ix, iy, value = L)

            
            self.rate.sleep()


try:
    map_node = Mapping()
    map_node.spin()
except rospy.ROSInterruptException:
    pass
