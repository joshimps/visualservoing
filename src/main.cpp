#include "ros/ros.h"
#include "robotController.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "visual_servoing");
  Robot robot();
  ros::NodeHandle nh;
  ros::spin();
  ros::shutdown();

  return 0;
}