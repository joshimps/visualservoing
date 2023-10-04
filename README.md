# Visual Servoing of a Handheld RGB-D Camera

The aim of this project is to use visual servoing techniques to control a Robot-held (Dobot or UR3) camera. 
The students can choose any pattern, and design any desired position on the image. 
When moving a fiducial/marker/object, the robot arm moves to point towards the fiducial/marker/object.

## Architecture and Equipment:
- UR3 Robot
- RGB-D Camera System
- C++
- MATLAB
- ROS
- ROS Toolbox for MATLAB

## Sensors and Control for Mechatronic Systems Specifics

## Industrial Robotics Specifics

## Dependent Packages
- Universal Robot Package - https://github.com/ros-industrial/universal_robot.git
- RealSense Gazebo Pluugin - https://github.com/pal-robotics/realsense_gazebo_plugin.git
- Intel realsense-ros - https://github.com/IntelRealSense/realsense-ros.git

## Launch D435 RGBD w/ ArUco Marker Gazebo
```
roslaunch sensors_control_sim d435_arUco.launch
```
## Launch UR3 w/ Mounted D435 RGBD & ArUco Marker Gazebo
```
roslaunch sensors_control_sim ur3_d435.launch
```
