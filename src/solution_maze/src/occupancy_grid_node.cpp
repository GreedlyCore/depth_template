#include <ros/ros.h>

#include <iostream>
#include <math.h>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <Eigen/Geometry>

using namespace std;

// Sensor characteristic: Min and Max ranges of the beams
double Zmax = 5000, Zmin = 170;
// Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
double l0 = 0, locc = 0.4, lfree = -0.4;
// Grid dimensions
double gridWidth = 0.1, gridHeight = 0.1;
// Map dimensions
double mapWidth = 10, mapHeight = 10;
// Robot size with respect to the map 
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;
// Defining an l vector to store the log odds values of each cell
double robotX, robotY, robotTheta = 0;
double timeStamp;
double measurementData[8];
static uint32_t seq_number = 0;
// ros::Time map_load_time = ros::Time::init();

ros::Publisher pub_map;
ros::Subscriber sub_laserscan; 
ros::Subscriber sub_pose; 
ros::Subscriber sub_point_cloud;

vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;

    // Compute r and phi
    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    double phi = atan2(yi - y, xi - x) - theta;

    // Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    // Evaluate the three cases
    if (r > min((double)Zmax, Zk + alpha/2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin)
        return l0;
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2)
        return locc;
    else if (r <= Zk)
        return lfree;
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    for (int i = 0; i < mapWidth/gridWidth; i++){
        for (int j = 0; j < mapHeight/gridHeight; j++){
            double xi = i*gridWidth + gridWidth/2 - robotXOffset;
            double yi = -(j*gridHeight + gridHeight/2) + robotYOffset;
            
            if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax)
                l[i][j] = l[i][j] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
        }    
    }
}

void laserscan_cb (const sensor_msgs::LaserScanConstPtr& laserscan_msg)
{
    // measurementData;
}

// create a buffer ???
void pose_cb (const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    
    robotX = pose_msg->pose.pose.position.x;
    robotY = pose_msg->pose.pose.position.x;

    // Extract orientation data and convert quaternion to angle in radians
    tf2::Quaternion orientation;
    tf2::convert(pose_msg->pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    robotTheta = yaw;

    occupancyGridMapping(robotX, robotY, robotTheta * (M_PI / 180), measurementData);

    // pub_map.publish()
}


// Function to project 3D point cloud to 2D and create occupancy grid
nav_msgs::OccupancyGrid updateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //     Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
    // rotation_matrix(0, 0) = 0.0;
    // rotation_matrix(0, 1) = 1.0;
    // rotation_matrix(1, 0) = -1.0;
    // rotation_matrix(1, 1) = 0.0;
    // Define the occupancy grid parameters
    double resolution = 0.1; // meters
    double width = 60.0; // meters
    double height = 60.0; // meters
    double origin_x = -width / 2.0;
    double origin_y = -height / 2.0;

    //   // Create a transformation matrix
    // Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    // transformation_matrix.block<3, 3>(0, 0) = rotation_matrix.block<3, 3>(0, 0);

    // // Create a new point cloud to store the rotated point cloud data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

    // // Rotate the point cloud data
    // pcl::transformPointCloud(*cloud, *rotated_cloud, transformation_matrix);

    // Convert the occupancy grid vector to an int8_t array

    // Create the occupancy grid message
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.seq = seq_number++;
    grid.header.stamp  = ros::Time::now(); 
    // grid.info.map_load_time = map_load_time;
    grid.info.resolution = resolution;
    grid.info.width = width / resolution;
    grid.info.height = height / resolution;
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 0.0;
    grid.data.resize(grid.info.width * grid.info.height, 0);

    for (int i = 0; i < grid.info.width; ++i) {
        for (int j = 0; j < grid.info.height; ++j) {
            double probability = l[i][j]; // Get the probability value
            int8_t occupancy_value;
            if (probability == l0) {
                occupancy_value = -1; // Unknown space
            } else {
                occupancy_value = static_cast<int8_t>(std::round((probability / 1.0) * 100.0));
                if (occupancy_value < 0) occupancy_value = 0;
                if (occupancy_value > 100) occupancy_value = 100;
            }
            grid.data[j * grid.info.width + i] = occupancy_value;
        }
    }

    // Project the point cloud onto the occupancy grid
    // for (const auto& point : cloud->points)
    // {
    //     int x = (point.x - origin_x) / resolution;
    //     int y = (point.y - origin_y) / resolution;
    //     if (x >= 0 && x < grid.info.width && y >= 0 && y < grid.info.height)
    //     {
    //         int index = y * grid.info.width + x;
    //         grid.data[index] = 100; // occupied
    //     }
    // }

    return grid;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // ROS_INFO_STREAM("Image width: " << cam->ImageWidth());
    ROS_INFO_STREAM("POSE: " << robotX << " " << robotY << " " << robotTheta);

    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create occupancy grid
    // robotX = 0; robotY = 0; robotTheta = 0;
    

    occupancyGridMapping(robotX, robotY, robotTheta * (M_PI / 180), measurementData);
    ROS_INFO_STREAM("computed, Inside cloud_cb!!!");
    nav_msgs::OccupancyGrid grid = updateOccupancyGrid(cloud);
    pub_map.publish(grid);
    // Convert occupancy grid to image
    // cv::Mat image = occupancyGridToImage(grid);
    // // Save image
    // cv::imwrite("map.png", image);
}

int main (int argc, char** argv)
{    
    // Initialize ROS
    ros::init (argc, argv, "map_maker_node");
    ros::NodeHandle nh;


    // sub_pose = nh.subscribe("amcl_pose", 10, pose_cb);
    sub_pose = nh.subscribe("orb_slam2_rgbd", 10, pose_cb);
    sub_laserscan = nh.subscribe("scan_filtered", 10, laserscan_cb);
    sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/output_noisy", 1, cloud_cb);

    pub_map = nh.advertise<nav_msgs::OccupancyGrid> ("output_map", 10);

    
    // remove from callback because it's exspensive operation
    // occupancyGridMapping(robotX, robotY, robotTheta * (M_PI / 180), measurementData);

    ros::spin ();
    
    return 0;
}