#include "ros/ros.h"
#include "robotController.h"
#include "robot.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "visual_servoing");

  ros::NodeHandle nh;

  Robot robot(nh);
  RobotController controller(nh)

  ros::spin();
  ros::shutdown();

  return 0;
}