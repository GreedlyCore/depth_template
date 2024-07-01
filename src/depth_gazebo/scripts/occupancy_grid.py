#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped, Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

from utils import prob_to_log_odds, log_odds_to_prob, linear_mapping_of_values

# Constants
FLOOR_SIZE_X = 25  # meters
FLOOR_SIZE_Y = 25  # meters
RESOLUTION = 0.1  # meters per cell
WORLD_ORIGIN_X = -FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = -FLOOR_SIZE_Y / 2.0
MAP_SIZE_X = int(FLOOR_SIZE_X / RESOLUTION)
MAP_SIZE_Y = int(FLOOR_SIZE_Y / RESOLUTION)
PRIOR_PROB = 0.5
OCC_PROB = 0.8
FREE_PROB = 0.2

class Mapping:
    def __init__(self):
        rospy.init_node('mapping_node', anonymous=False)
        rospy.loginfo("creation of the map  has been started.")

        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to LaserScan topic
        rospy.Subscriber('/scan_filtered', LaserScan, self.__process_scan)
        rospy.Subscriber('/odom', Odometry, self.__process_odom)

        self.robot_x = 0
        self.robot_y = 0
        # Initialize map publisher
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=1 )

        # Initialize the occupancy grid message
        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'map'
        self.occupancy_grid_msg.info = MapMetaData(
            width=MAP_SIZE_X,
            height=MAP_SIZE_Y,
            resolution=RESOLUTION,
            origin=Pose(
                position=Point(x=WORLD_ORIGIN_X, y=WORLD_ORIGIN_Y)
            )
        )

        self.map = self.__generate_map()
        self.pub_map()
        rospy.Timer(rospy.Duration(1), self.pub_map)

    def __generate_map(self):
        """
        Initialize map in the odom frame.
        """
        rospy.loginfo(f"Generating map of size: {(MAP_SIZE_X, MAP_SIZE_Y)}")
        map = prob_to_log_odds(PRIOR_PROB) * np.ones((MAP_SIZE_X, MAP_SIZE_Y))
        return map

    def mark_map(self, ix, iy, value):
        """
        Update a map's cell value.
        """
        try:
            self.map[ix, iy] = value
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

    def pub_map(self, event=None):
        """
        Publish an OccupancyGrid to the /map topic.
        """
        map = log_odds_to_prob(self.map)
        map = linear_mapping_of_values(map)  # To visualize in Rviz

        self.occupancy_grid_msg.header.stamp = rospy.Time.now()
        self.occupancy_grid_msg.data = map.astype(int).T.reshape(map.size, order='C').tolist()  # row-major order
        self.map_publisher.publish(self.occupancy_grid_msg)

    def __odom_coords_to_2d_array(self, x, y):
        """
        Transform a coord in the odom frame to coords in a 2D array.
        """
        ix = int(x / RESOLUTION) + MAP_SIZE_X // 2
        iy = int(y / RESOLUTION) + MAP_SIZE_Y // 2
        return ix, iy

    def __get_perceptual_range(self, origin, target):
        """
        Get the grid cells that belong to the line between the origin and target.
        The returned points' coordinates are int-indexes of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
        """
        x0, y0 = self.__odom_coords_to_2d_array(origin.point.x, origin.point.y)
        x1, y1 = self.__odom_coords_to_2d_array(target.point.x, target.point.y)

        rospy.loginfo(f"checking input1: ({origin}) \n&&\n  ({target})")

        rospy.loginfo(f"checking input2: ({x0},{y0}) && ({x1},{y1})")
        

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

    def __process_odom(self, msg):
        self.robot_x =  msg.pose.pose.position.x 
        self.robot_y =  msg.pose.pose.position.y 
        

    def __process_scan(self, msg):
        """
        Process a new LaserScan message and update map.
        """
        from_frame = msg.header.frame_id
        to_frame = 'odom'
        now = msg.header.stamp
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))

        for i, range_value in enumerate(msg.ranges):
            if np.isinf(range_value) or np.isnan(range_value):
                continue
            angle = angle_min + i * angle_increment
            laser_point = PointStamped()
            laser_point.point.x = range_value * np.cos(angle)
            laser_point.point.y = range_value * np.sin(angle)
            laser_point.header.frame_id = from_frame
            laser_point.header.stamp = now
            laser_base_point = PointStamped()
            laser_base_point.header.frame_id = from_frame
            laser_base_point.header.stamp = now
            laser_base_point.point.x = self.robot_x
            laser_base_point.point.y = self.robot_y
            laser_base_point.point.z = 0    
            lp_in_odom_frame = do_transform_point(laser_point, transform)
        
            
            perceptual_range = self.__get_perceptual_range(
                laser_base_point, lp_in_odom_frame
            )

            rospy.loginfo(f"info message: {perceptual_range}")
            

            for j, (ix, iy) in enumerate(perceptual_range):
                p = self.inverse_range_sensor_model(j, len(perceptual_range))
                l_prev = self.get_map_val(ix, iy)
                if l_prev is None: continue
                l = l_prev + prob_to_log_odds(p) - prob_to_log_odds(PRIOR_PROB)
                self.mark_map(ix, iy, value=l)

    def inverse_range_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        This is a naive implementation.
        """
        if i == (len_perceptual_range - 1):
            return OCC_PROB
        else:
            return FREE_PROB

if __name__ == '__main__':
    try:
        map_node = Mapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
