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

## USAGE
### Launch UR3 w/ Mounted D435 RGBD & ArUco Marker Gazebo
```
roslaunch ur3_gazebo ur_gripper_85_cubes.launch ur_robot:=ur3 grasp_plugin:=1
```
### Launch Aruco Ros Node
```
```

### Launch Visual Servoing Node
```
```
