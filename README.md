
# Building an environment map using the realsence d435 camera
## ITMO practice, second year robotics

Here you can see a workspace with all essential package, other dependencies you can instal via classic commands:
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
You can test grid mapping in gazebo using _depth_gazebo_ package. Package provides you with omni wheel robot with depth camera on it,
you can ride it throught _teleop_ package:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Manual installation of packages & dependencies
- EASY: Run it from your catkin folder

```
sh src/depth_gazebo/install_dependencies.sh
```
- EXPERT: install by yourself...

- **orb_slam2_ros** package
```
git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
```
- **depthimage_to_laserscan** package
```
git clone -b melodic-devel https://github.com/ros-perception/depthimage_to_laserscan.git
```
- **realsense2_description** - took from
- **realsense_gazebo_plugin** - took from

### Useful commands
- don't forget to build && source it it
```
catkin build && source ./devel/setup.bash
```

- You can save map via cmd:
```
rosrun map_server map_saver --occ 90 --free 10 -f mymap map:=/best_map
```

## Prepare lidar
Currently we are using DTOF LIDAR LD19
...


