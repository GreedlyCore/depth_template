#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/io.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // convert to PCLPointCloud2
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_msg, *cloud_blob);

  // filtering (voxelization)
  pcl::PCLPointCloud2Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(0.1, 0.1, 0.1); // Leaf size --> 3d boxes that approx you
  sor.filter(*cloud_filtered_blob);

  // convert to PointCloud<T>
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
  
  // segment planar model
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  // The four coefficients of the plane are its Hessian Normal form: [normal_x normal_y normal_z d]  
  seg.setModelType(pcl::SACMODEL_PLANE); // used to determine plane models. 
  seg.setMaxIterations(1500); //1000-->1200-->1500
  seg.setDistanceThreshold(0.08); //0.08 --> 0.05
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  // remove inliers ~ remove detected ground
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_outliers);

  // Ax + By + Cz + D = 0 plane equation to be projected into
  // Create a set of planar coefficients -> [0, 1, 2, 3] <-> [A, B, C, D]
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients ());
  coefficients2->values.resize (4);
  coefficients2->values[0] = 0.0;
  coefficients2->values[1] = 1.0;
  coefficients2->values[2] = 0.0;
  coefficients2->values[3] = 0.0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_outliers);
  proj.setModelCoefficients (coefficients2);
  proj.filter (*cloud_projected);

  // convert to sensor_msgs::PointCloud2
  pcl::PCLPointCloud2::Ptr cloud2_outliers (new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*cloud_projected, *cloud2_outliers);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud2_outliers, output);
  pub.publish(output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_ground_removal");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  ros::spin ();
}