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

std::vector<tf2Scalar> RobotController::calculateEndEffectorVelocity(){
    //First calculate the error between end effector position and the desired position

    robot_->calculateJointTransforms();
    tf2::Transform tBaseToEndEffector = robot_->getEndEffectorTransform();
    tf2::Transform tError = fiducialPoseLocal_ * (tBaseToEndEffector.inverse());
    tf2::Vector3 positionError = tError.getOrigin();
    tf2::Quaternion rotationalError = tError.getRotation();
    
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
    tf2::Vector3 fiducialTranslation(fiducialPoseWithCovarianceLocal_.pose.position.x,fiducialPoseWithCovarianceLocal_.pose.position.y,fiducialPoseWithCovarianceLocal_.pose.position.z);
    tf2::Quaternion fiducialRotation(fiducialPoseWithCovarianceLocal_.pose.orientation.x,
                                     fiducialPoseWithCovarianceLocal_.pose.orientation.y,
                                     fiducialPoseWithCovarianceLocal_.pose.orientation.z,
                                     fiducialPoseWithCovarianceLocal_.pose.orientation.w);
    
    fiducialPoseLocal_.setOrigin(fiducialTranslation);
    fiducialPoseLocal_.setRotation(fiducialRotation);
}
