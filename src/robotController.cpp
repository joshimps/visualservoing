#include "robotController.h"


///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////

RobotController::RobotController(ros::NodeHandle nh, Robot* robot, double gain, double errorThreshold){
    nh_ = nh;
    robot_ = robot;
    gain_ = gain;
    errorThreshold_ = errorThreshold;
    //fiducialPositionSub_ = nh_.subscribe("fiducialPositionTopic", 1000, &RobotController::fiducialPositionCallBack, this);
    //jointVelocityPub_ = nh_.publish("jointVelocityTopic", 3, false);
}


///////////////////////////////////////////////////////////
// Calculation
//////////////////////////////////////////////////////////

std::vector<tf2Scalar> RobotController::calculateEndEffectorVelocity(){
    //First calculate the error between end effector position and the desired position

    robot_->calculateJointTransforms();
    Eigen::MatrixXd tError = fiducialPoseLocal_
    tf2::Vector3 positionError = 
    tf2::Quaternion rotationalError = 
    
    std::vector<tf2Scalar> endEffectorVelocity;

    endEffectorVelocity.at(0) = gain_ * positionError.x();
    endEffectorVelocity.at(1) = gain_ * positionError.y();
    endEffectorVelocity.at(2) = gain_ * positionError.z();
    endEffectorVelocity.at(3) = gain_ * rotationalError.x();
    endEffectorVelocity.at(4) = gain_ * rotationalError.y();
    endEffectorVelocity.at(5) = gain_ * rotationalError.z();

    return endEffectorVelocity;
}

///////////////////////////////////////////////////////////
// Callbacks and Services
//////////////////////////////////////////////////////////

void RobotController::fiducialPositionCallBack(geometry_msgs::PoseWithCovariancePtr &msg){
    geometry_msgs::PoseWithCovariance fiducialPoseWithCovarianceLocal_ = *msg;
    Eigen::MatrixXd fiducialTranslation(3,1);
    Eigen::Quaterniond fiducialRotation;

    //Row 1
    fiducialTranslation(0,0) = fiducialPoseWithCovarianceLocal_.pose.position.x;
    //Row 2
    fiducialTranslation(1,0) = fiducialPoseWithCovarianceLocal_.pose.position.y;
    //Row 3
    fiducialTranslation(2,0) = fiducialPoseWithCovarianceLocal_.pose.position.z;

    fiducialRotation
    


    (fiducialPoseWithCovarianceLocal_.pose.orientation.x,
                                     fiducialPoseWithCovarianceLocal_.pose.orientation.y,
                                     fiducialPoseWithCovarianceLocal_.pose.orientation.z,
                                     fiducialPoseWithCovarianceLocal_.pose.orientation.w);
    
    fiducialPoseLocal_.setOrigin(fiducialTranslation);
    fiducialPoseLocal_.setRotation(fiducialRotation);
}
