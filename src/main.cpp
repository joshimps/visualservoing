#include "ros/ros.h"
#include "ros/console.h"
#include "robotController.h"
#include <thread>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servoing");
  ros::NodeHandle nh;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // DH Parameters of a UR3 robot are provided
  std::vector<double> d{0.1519, 0, 0, 0.11235, 0.08535, 0.0819};
  std::vector<double> a{0, -0.24365, -0.21325, 0, 0, 0};
  std::vector<double> alpha{M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
  std::vector<std::string> jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

  double gain = 0.3;
  double errorThreshold = 0.001;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  Robot robot(nh, d, a, alpha, jointNames);
  RobotController robotController(nh, &robot, gain, errorThreshold);

  ros::waitForShutdown();

  return 0;
}