# Visual Servoing of a Handheld RGB-D Camera

The aim of this project is to use visual servoing techniques to control a Robot-held (Dobot or UR3) camera. 
The students can choose any pattern, and design any desired position on the image. 
When moving a fiducial/marker/object, the robot arm moves to point towards the fiducial/marker/object.

## Architecture and Equipment:
- UR3 Robot
- RGB-D Camera System
- C++
- ROS Noetic
- Python 3

## Dependent Packages
[UR3 Simulator with Camera and Gripper](https://github.com/sheepskins/sandc_simulation)

[image pipeline](https://github.com/ros-perception/image_pipeline)

[aruco_ros](https://github.com/pal-robotics/aruco_ros)

[realsense2_ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)

[realsense2_gazebo](https://github.com/pal-robotics/realsense_gazebo_plugin/tree/melodic-devel)

## USAGE
### Launch UR3 w/ Mounted D435 RGBD & ArUco Marker Gazebo
```
roslaunch visual_servoing ur3.launch
```
### Launch Aruco Ros Node

Update markerId and markerSize (in metres) in aruco.cfg if required

```
roslaunch visual_servoing aruco_marker_finder.launch
```

### Launch Visual Servoing Node
```
rosrun visual_servoing visual_servoing
```

### Launch RViz Visual Servoing Simulation
```
roslaunch visual_servoing cameraRviz.launch
```
