#include <ros/ros.h>

#include <iostream>
#include <math.h>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
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
double robotX, robotY, robotTheta;
double timeStamp;
double measurementData[8];

ros::Publisher pub_map;
ros::Subscriber sub_laserscan; 
ros::Subscriber sub_pose; 

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

int main (int argc, char** argv)
{    
    // Initialize ROS
    ros::init (argc, argv, "map_maker_node");
    ros::NodeHandle nh;


    sub_pose = nh.subscribe("amcl_pose", 10, pose_cb);
    sub_laserscan = nh.subscribe("scan", 10, laserscan_cb);

    pub_map = nh.advertise<nav_msgs::OccupancyGrid> ("output_map", 10);

    
    // remove from callback because it's exspensive operation
    // occupancyGridMapping(robotX, robotY, robotTheta * (M_PI / 180), measurementData);

    ros::spin ();
    
    return 0;
}