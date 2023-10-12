#include "ros/ros.h"
#include "robotController.h"
#include <thread>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servoing");
  ros::NodeHandle nh;
  
  std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
  std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
  std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
  
  double gain = 0.5;
  double errorThreshold = 0.1;


  ros::AsyncSpinner spinner(0);
  spinner.start();


  Robot robot(nh,d,a,alpha);
  RobotController robotController(nh,&robot,gain,errorThreshold);

  
  ros::waitForShutdown();

  return 0;
}