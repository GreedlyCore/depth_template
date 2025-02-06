
# Building an environment map using the realsence d435 camera
## ITMO practice, second year robotics

<p align="center">
<img src = "Docs/gazebo.jpg?raw=true" width="70%"/>
</p>

ROS_DISTRO: Noetic

This repo is only a main package, other ones you need to install to the same workspace via classic commands:
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Version 1 - mapping with diffdrive + lidar + localization from gazebo

We need to start from simple example, diffdrive car is pretty universal, lidar point is ready to use without any additional filtering, localization from 
gazebo because for mapping task we need to know exact pose of the robot + all beam lengths at exact moment.
run with:
```
roslaunch depth_gazebo diff_lidar_naive.launch
```
below is overlapped - ground truth (screenshot from gazebo) and scaled+rotated a bit occupancy grid
<p align="center">
<img src = "Docs/overlap_map_v1.jpg?raw=true" width="50%"/>
</p>

## Version 2 - mapping with diffdrive + d435 + localization from gazebo

run with:
```
roslaunch depth_gazebo diff_d435_naive.launch
```

## Version 3 - mapping with diffdrive + d435 + localization from gazebo + noise

run with:
```
roslaunch depth_gazebo diff_d435_noisy.launch
```

## Version 3.1 - mapping with omnidrive + d435 + localization from gazebo + noise

run with:
```
roslaunch depth_gazebo diff_d435_noisy.launch
```

## Compare with ground truth 

scale and record ground truth
how to compare? using teleop create map or using explore package

## Compare with other solutions

Also you can compare it with [gmapping](http://wiki.ros.org/gmapping) 'cartographer' with:
```
roslaunch depth_gazebo diff_gmapping.launch
```
Also you can compare it with [google cartographer](https://github.com/cartographer-project/cartographer_ros) with:
```
roslaunch depth_gazebo diff_google.launch
```
You can do the same things but with omnidirectional wheeled platform, commands will be almost the same (still unstable):
```
roslaunch depth_gazebo omni_naive.launch
roslaunch depth_gazebo omni_gmapping.launch
roslaunch depth_gazebo omni_google.launch
```

## Manual installation of packages & dependencies
- Run it from your catkin workspace folder
```
sh src/depth_gazebo/install_dependencies.sh
```
it provides you with

- **orb_slam2_ros** that gives odom->base_link transform
- **depthimage_to_laserscan** that turn poincloud without ground plane into laser scan msg format
- **realsense2_description**+**realsense_gazebo_plugin** - obviously, for simulating d435, but i fitted real params, because in that repo was wrong FOV's and distances
- **pcl_ros** - to work with pointcloud - filter it, rotating, some basic operations
- **amcl** - estimate robot's localization (/amcl_pose)

You can test grid mapping in gazebo using _depth_gazebo_ package. It provides you with omni wheel robot with depth camera on it,
you can ride it throught _teleop_ package:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Useful commands
- Don't forget to build && source it 
```
catkin build && source ./devel/setup.bash
```
- check how fast some topic is posting
```
rostopic hz /TOPIC_NAME
```
- special flags for debugging big messages via `rostopic echo`: --noarr , --nostr
- You can tune algorithm parameters online with [Dynamic Reconfigure](http://wiki.ros.org/dynamic_reconfigure/Tutorials) rqt plugin
- You can save map via cmd:
```
rosrun map_server map_saver --occ 90 --free 10 -f FILE_MAP_NAME map:=/map
```
- See TF tree
```
rosrun rqt_tf_tree rqt_tf_tree
```


## Acknowledgement

My main references is here
- [ManuelZ's solution](https://github.com/ManuelZ/robotics_algorithms/tree/main) - he implemented pretty similar algorithm, but for one laser beam
- [bmaxdk's solution](https://github.com/bmaxdk/ros-noetic-where-am-i-amcl) - pretty cool example for working with AMCL package with ROS Noetic
- [Aleksa Lukovic's solution](https://github.com/lukovicaleksa/grid-mapping-in-ROS) - nice drawing of map realization, but without needed transforms and using OccupancyGrid messages
- [Nice paper on lidar inverse model](http://www.diva-portal.org/smash/get/diva2:1900124/FULLTEXT01.pdf)

But i don't recommend to took any parts of code from any of them - write yuor own code, becuase all mistakes and misunderstood will raise from stealing without understanding.
