This package is base on [unitree_ros_to_real v3.8.0](https://github.com/unitreerobotics/unitree_ros_to_real/tree/v3.8.0), supporting only the Unitree Go1 quadruped.


# Introduction
This ROS package provides high level control of the Unitree Go1 quadrupted, exposes the IMU data in /imu/data and also fixes the issue where tf is missing in the /odom message.  

## Environment
We recommand users to run this package in Ubuntu 18.04 and ROS melodic environment

## Dependencies
[unitree_legged_sdk v3.8.0](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/3.8.0)

# Install
```
git clone -b v3.8.0 https://github.com/unitreerobotics/unitree_legged_sdk.git ~/catkin_ws/src/unitree_legged_sdk
git clone https://github.com/ICE9-Robotics/unitree_ros_to_real.git ~/catkin_ws/src/unitree_ros_to_real
```

Then build the package:
```
cd ~/catkin_ws
catkin_make
```

# Use
Follow [this guide](https://github.com/ICE9-Robotics/ice9_unitree/wiki/Access-the-Unitree-PCs) to connect to the Unitree network, then:
```
roslaunch unitree_legged_real real.launch
```
