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
        rospy.loginfo("creation of the map  has been started.")
        self.rate = rospy.Rate(50) 
        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # for scan msg
        self.from_frame = "lidar_link" #"camera_bottom_screw_frame" 
        self.to_frame = "map"

        self.robot_name = 'my_robot_model'
        self.ranges = []
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
        # already divided by two
        self.alpha = 0.05
        self.beta = 0.3 # angle in rads (???) # the opening angle of this sensor is

        rospy.Subscriber('/scan_filtered', LaserScan, self.get_scan, queue_size=1)
        # rospy.Subscriber('/odom', Odometry, self.get_odom, queue_size=3)  
        rospy.Subscriber('/ground/state', Odometry, self.get_gazebo, queue_size=1)  
        # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_pose, queue_size=1)  
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

    def get_odom(self, msg):
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y 
        self.robot_theta = transform_orientation(msg.pose.pose.orientation)
    
    # in earth frame, but --> map
    def get_gazebo(self, msg):
        # rospy.loginfo(f"GAZEBO:\n X: {msg.pose[-1].position.x }\nY: {msg.pose[-1].position.y }\n X: {msg.pose[-1].position.x }\n THETA: {np.round(transform_orientation(msg.pose[-1].orientation), 3)}")        
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y 
        self.robot_theta =  transform_orientation(msg.pose.pose.orientation)
            
    def get_scan(self, msg):
        """
        get a new LaserScan message
        """
        self.ranges = msg.ranges

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
        # print(f"ANGLES: r = {r} phi = {phi} k = {k} \n POSES: ({x}, {y}) , ({self.robot_pixel_x}, {self.robot_pixel_y})\n")
        # assert r > 0 and phi != 0, f"(r, phi) == (negative , 0)"
        
        if not np.isnan(ranges[k]):
            # rospy.loginfo(f"1st: {r} > {min(self.range_max, np.round(self.ranges[k], 4) + self.alpha)} or {np.round(abs( angle_diff( phi, self.angles[k]) ), 5)} > {self.beta}")
            # rospy.loginfo(f"2nd: {ranges[k]} < {self.range_max} and {abs(r - ranges[k])} < {self.alpha}")
            # rospy.loginfo(f"3rd: {r} <= {ranges[k]}")
            if r > min(self.range_max, ranges[k] + self.alpha) or abs( angle_diff( phi, self.angles[k]))> self.beta:
                return self.l0 # out of range case
            if ranges[k] < self.range_max and abs(r - ranges[k]) < self.alpha:
                return self.OCC_L # if range for cell is over  ( +-self.alpha ) than z^k_t - our measurement
            if r <= ranges[k]:
                return self.FREE_L # else case
    def spin(self):
        rospy.sleep(5)
        while not rospy.is_shutdown():
            self.pub_map()
            for i, range_value in enumerate(self.ranges):
                if np.isinf(range_value) or np.isnan(range_value):
                    continue

                # laser_point - one dot from laserscan in camera_frame
                # base_point - estimated robot position via /amcl_pose
                # provide projections --> we need 4x1 vector because we have 4x4 matrix
                
                perceptual_range = self.get_perceptual_range(
                    [self.robot_x, self.robot_y, self.robot_theta], [range_value, self.angles[i]])
                for (ix, iy) in perceptual_range:
                    L = self.inverse_range_sensor_model(ix, iy, [self.robot_x, self.robot_y, self.robot_theta], self.ranges) - self.l0
                    self.mark_map(ix, iy, value = L)

            # print(f"ROBOT PIXEL:({self.robot_pixel_x}, {self.robot_pixel_y})\n POSES:{perceptual_range}")
            # print("NEXT...\n")
            # print(f"ROBOT THETA: {self.robot_theta}")
            # print(f"L is: {L}")
            
            self.rate.sleep()


try:
    map_node = Mapping()
    map_node.spin()
except rospy.ROSInterruptException:
    pass
