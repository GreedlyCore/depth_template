// Gaussian noise to pcl
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


#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

ros::Publisher pub;
ros::Subscriber sub;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void addGaussiaNoise(const sensor_msgs::PointCloud2::ConstPtr &input, sensor_msgs::PointCloud2 &output,
                     double standard_deviation)
{
    TicToc tt;
    tt.tic();

    print_highlight("Adding Gaussian noise with mean 0.0 and standard deviation %f\n", standard_deviation);

    PointCloud<PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr output_cloud_pcl(new pcl::PointCloud<PointXYZ>());

    fromROSMsg(*input, *xyz_cloud);

    PointCloud<PointXYZ>::Ptr xyz_cloud_filtered(new PointCloud<PointXYZ>());
    xyz_cloud_filtered->points.resize(xyz_cloud->points.size());
    xyz_cloud_filtered->header = xyz_cloud->header;
    xyz_cloud_filtered->width = xyz_cloud->width;
    xyz_cloud_filtered->height = xyz_cloud->height;

    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, standard_deviation);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<>> var_nor(rng, nd);

    for (size_t point_i = 0; point_i < xyz_cloud->points.size(); ++point_i)
    {
        xyz_cloud_filtered->points[point_i].x = xyz_cloud->points[point_i].x + static_cast<float>(var_nor());
        xyz_cloud_filtered->points[point_i].y = xyz_cloud->points[point_i].y + static_cast<float>(var_nor());
        xyz_cloud_filtered->points[point_i].z = xyz_cloud->points[point_i].z + static_cast<float>(var_nor());
    }

    sensor_msgs::PointCloud2 input_xyz_filtered;

    concatenateFields(*xyz_cloud, *xyz_cloud_filtered, *output_cloud_pcl);

    toROSMsg(*output_cloud_pcl, output);

    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms: ");
    print_value("%d", output.width * output.height);
    print_info(" points]\n");
}

void noise_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    sensor_msgs::PointCloud2 output_noisy;
    //   addGaussiaNoise(const sensor_msgs::PointCloud2::ConstPtr &input, sensor_msgs::PointCloud2 &output,
    //                      double standard_deviation)
    addGaussiaNoise(cloud_msg, output_noisy, 0.01);
    pub.publish(output_noisy);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rgbd_add_noise");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe("/output", 1, noise_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output_noisy", 1);

    ros::spin();
}