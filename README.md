This package is base on [unitree_ros_to_real v3.8.0](https://github.com/unitreerobotics/unitree_ros_to_real/tree/v3.8.0), supporting only the Unitree Go1 quadruped.

# Introduction
This ROS package provides high level control of the Unitree Go1 quadrupted and exposes the IMU data in /imu/data.  

## Environment
We recommand users to run this package in Ubuntu 18.04 and ROS melodic environment

## Dependencies
[unitree_legged_sdk v3.8.0](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/3.8.0)

# Configuration
Before compiling this package, please download the corresponding unitree_legged_sdk as noted above, and put it to your own workspace's source folder(e.g. `~/catkin_ws/src`). Be careful with the sdk folder name. It should be "unitree_legged_sdk" without version tag.

# Build
You can use catkin_make to build ROS packages. First copy the package folder to `~/catkin_ws/src`, then:
```
cd ~/catkin_ws
catkin_make
```

# Run the package
Follow [this guide](https://github.com/ICE9-Robotics/ice9_unitree/wiki/Access-the-Unitree-PCs) to connect to the Unitree network, then:
```
roslaunch unitree_legged_real real.launch
```
