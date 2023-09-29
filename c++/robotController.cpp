#include "robotController.h"


///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////

RobotController::RobotController(ros::NodeHandle nh, Robot* robot){
    nh_ = nh;
    robot_ = robot;
    //fiducialPositionSub_ = nh_.subscribe("fiducialPositionTopic", 1000, &robotController::fiducialPositionCallBack, this);
    //jointVelocityPub_ = nh_.publish("jointVelocityTopic", 3, false);
}


///////////////////////////////////////////////////////////
// Calculation
//////////////////////////////////////////////////////////

void RobotController::calculateJointVelocities(){
    //First calculate the error between end effector position and the desired position

    tf2::Transform tBaseToEndEffector = 
    tf2::Transform tError = fiducialPoseLocal_ * 
}

///////////////////////////////////////////////////////////
// Callbacks and Services
//////////////////////////////////////////////////////////

void RobotController::fiducialPositionCallBack(geometry_msgs::PoseWithCovariancePtr &msg){
    fiducialPoseLocal_ = *msg;
}
