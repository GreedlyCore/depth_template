# ATTENTION, non local install! point cloud library
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-pcl-ros
sudo apt install pcl-tools
sudo apt-get install libboost-all-dev 
# eigen lib
sudo apt-get install libeigen3-dev

sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
cd src
git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
git clone -b melodic-devel https://github.com/ros-perception/depthimage_to_laserscan.git
git clone https://github.com/issaiass/realsense2_description.git
git clone -b melodic-devel https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone -b noetic-devel https://github.com/introlab/rtabmap_ros.git
