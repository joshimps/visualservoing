# Visual Servoing of a Handheld RGB-D Camera

The aim of this project is to use visual servoing techniques to control a Robot-held (Dobot or UR3) camera and to provide visual feedback to the user for the required translational and rotational movements.

The students can choose any pattern, and design any desired position on the image.


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

## Usage in Simulation with UR3
### Launch UR3 w/ Mounted D435 RGBD & ArUco Marker Gazebo
```
roslaunch visual_servoing ur3.launch
```
### Launch Aruco Ros Node

Update markerId and markerSize (in metres) on command line if required

```
roslaunch visual_servoing aruco_robot.launch
```

### Launch Visual Servoing Node
```
rosrun visual_servoing visual_servoing
```
## Usage for handheld servoing 
### Start D435
```
roslaunch realsense_ros2 aligned_depth_to_marker.launch
```
### Launch Aruco Ros Node
```
roslaunch visual_servoing aruco_handheld.launch
```
### Launch RViz Visual Servoing Simulation
```
roslaunch visual_servoing cameraRviz.launch
```
## Statement of Contribution
All three collaboraters on this project, Joshua Impey, Joshua Schiff and Mitchell Rosser contributed equally to the codebase. Whilst each member focused on different areas, all switched tasks to help as required. 
