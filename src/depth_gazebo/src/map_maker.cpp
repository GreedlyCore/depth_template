#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
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


ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher pub_projected;




// Function to project 3D point cloud to 2D and create occupancy grid
nav_msgs::OccupancyGrid createOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
    rotation_matrix(0, 0) = 0.0;
    rotation_matrix(0, 1) = 1.0;
    rotation_matrix(1, 0) = -1.0;
    rotation_matrix(1, 1) = 0.0;
    // Define the occupancy grid parameters
    double resolution = 0.1; // meters
    double width = 60.0; // meters
    double height = 60.0; // meters
    double origin_x = -width / 2.0;
    double origin_y = -height / 2.0;

      // Create a transformation matrix
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix.block<3, 3>(0, 0);

    // Create a new point cloud to store the rotated point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

    // Rotate the point cloud data
    pcl::transformPointCloud(*cloud, *rotated_cloud, transformation_matrix);




    // Create the occupancy grid message
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = resolution;
    grid.info.width = width / resolution;
    grid.info.height = height / resolution;
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height, 0);

    // Project the point cloud onto the occupancy grid
    for (const auto& point : rotated_cloud->points)
    {
        int x = (point.x - origin_x) / resolution;
        int y = (point.y - origin_y) / resolution;
        if (x >= 0 && x < grid.info.width && y >= 0 && y < grid.info.height)
        {
            int index = y * grid.info.width + x;
            grid.data[index] = 100; // occupied
        }
    }

    return grid;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create occupancy grid
    nav_msgs::OccupancyGrid grid = createOccupancyGrid(cloud);
    pub.publish(grid);
    // Convert occupancy grid to image
    // cv::Mat image = occupancyGridToImage(grid);
    // // Save image
    // cv::imwrite("map.png", image);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_maker");
    ros::NodeHandle nh;

    sub = nh.subscribe<sensor_msgs::PointCloud2>("/output_noisy", 1, cloud_cb);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("output_map", 1) ;

    ros::spin();

    return 0;
}
