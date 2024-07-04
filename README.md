
# Building an environment map using the realsence d435 camera
## ITMO practice, second year robotics

<p align="center">
<img src = "Docs/gazebo.jpg?raw=true" width="70%"/>
</p>

ROS_VERSION: 1, Noetic

Here you can see a workspace with all essential packages, other dependencies (if needed) you can instal via classic commands:
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

you can run naive cartographer with:
```
roslaunch depth_gazebo main_diff_naive.launch
```
you can run gmapping cartographer with:
```
roslaunch depth_gazebo main_diff_gmapping.launch
```
(NOT WORKING YET) -- you can run google cartographer with:
```
roslaunch depth_gazebo main_google.launch
```
you can run naive cartographer with omni wheeled mobile robot (have got some problems, unstable):
```
roslaunch depth_gazebo main_google.launch
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
- don't forget to build && source it 
```
catkin build && source ./devel/setup.bash
```
- check if some topic is posting
```
rostopic hz /TOPIC_NAME
```
- special flags for debugging big messages via `rostopic echo`: --noarr , --nostr
- You can save map via cmd:
```
rosrun map_server map_saver --occ 90 --free 10 -f FILE_MAP_NAME map:=/map
```
- See TF tree
```
rosrun rqt_tf_tree rqt_tf_tree
```


## Acknowledgement

That's not 100% my code, i tryied to bring my not best solution throught pain & mistakes, so i took some references from that github repositories and im grateful to them:
- [ManuelZ's solution](https://github.com/ManuelZ/robotics_algorithms/tree/main) - he implemented pretty similar algorithm, but for one laser beam
- [bmaxdk's solution](https://github.com/bmaxdk/ros-noetic-where-am-i-amcl) - pretty cool example for working with AMCL package with ROS Noetic
- [Aleksa Lukovic's solution](https://github.com/lukovicaleksa/grid-mapping-in-ROS) - nice drawing of map realization, but without needed transforms and using OccupancyGrid messages

Mostly, i took the first and the last one solution and triyed to adapt it on my situation, for depth camera purposes
