#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <random>

ros::Publisher noisy_depth_image_pub_;
ros::Subscriber depth_image_sub_;
ros::Subscriber camera_info_sub_;
sensor_msgs::CameraInfo camera_info_;

void addGaussianNoise(cv::Mat& image)
{
    // Initialize the random number generator
    std::default_random_engine generator;
    // SETUP DISTRIBUTION HERE: Mean, standard deviation 
    std::normal_distribution<float> distribution(0.0, 0.07); 

    // Add  noise to each pixel
    for (int i = 0; i < image.rows; ++i)
    {
        for (int j = 0; j < image.cols; ++j)
        {
            if (std::isfinite(image.at<float>(i, j))) // Check if the pixel is valid
            {
                image.at<float>(i, j) += distribution(generator);
            }
        }
    }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the ROS image message to a cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Add Gaussian noise to the depth image
    addGaussianNoise(cv_ptr->image);

    // Convert the cv::Mat back to a ROS image message
    sensor_msgs::ImagePtr noisy_image_msg = cv_ptr->toImageMsg();

    // Publish the noisy depth image
    noisy_depth_image_pub_.publish(noisy_image_msg);
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    // Store the camera info if needed
    camera_info_ = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage_add_noise");
    ros::NodeHandle nh;

    depth_image_sub_ = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallback);
    camera_info_sub_ = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);
    noisy_depth_image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/depth/image_noisy", 1);

    ros::spin();
}
