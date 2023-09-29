#include "ros/ros.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "visualServoing");

  ros::NodeHandle nh;

  ros::spin();
  ros::shutdown();

  return 0;
}