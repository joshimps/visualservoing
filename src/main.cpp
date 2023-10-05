#include "ros/ros.h"
#include "robotController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servoing");

  ros::NodeHandle nh;
  std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
  std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
  std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
  Robot robot(nh,d,a,alpha);
  
  //Lets set the joint angles and compare with values obtained from Matlab
  robot.calculateJointTransforms();
  robot.calculateJointTransformsToBase();
  robot.calculateJacobian();

  double gain = 0.1;
  double errorThreshold = 1;

  RobotController robotController(nh,&robot,gain,errorThreshold);

  robotController.moveRobot();
  
  ros::spin();
  ros::shutdown();

  return 0;
}