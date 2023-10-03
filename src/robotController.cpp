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
// Actions
//////////////////////////////////////////////////////////

void RobotController::moveRobot(){
    std::unique_lock<std::mutex> lck(fiducialPoseMutex_);
    Eigen::MatrixXd endEffectorVelocity(6,1);
    Eigen::MatrixXd jointVelocities;

    while(endEffectorVelocity.norm() > errorThreshold_){
    
        endEffectorVelocity(0,0) = gain_ * fiducialTranslationLocal_(0,0);
        endEffectorVelocity(1,0) = gain_ * fiducialTranslationLocal_(1,0);
        endEffectorVelocity(2,0) = gain_ * fiducialTranslationLocal_(2,0);
        endEffectorVelocity(3,0) = gain_ * fiducialRotationLocal_.x();
        endEffectorVelocity(4,0) = gain_ * fiducialRotationLocal_.y();
        endEffectorVelocity(5,0) = gain_ * fiducialRotationLocal_.z();

        //Calculate the jacobian of the current pose 
        robot_->calculateJointTransforms();
        robot_->calculateJacobian();

        jointVelocities = robot_->getJacobian().completeOrthogonalDecomposition().pseudoInverse() * endEffectorVelocity;
        
        //Publish the joint velocities to the robot here
    }
    
}

///////////////////////////////////////////////////////////
// Callbacks and Services
//////////////////////////////////////////////////////////

void RobotController::fiducialPositionCallBack(geometry_msgs::PoseWithCovariancePtr &msg){

    std::unique_lock<std::mutex> lck(fiducialPoseMutex_);

    geometry_msgs::PoseWithCovariance fiducialPoseWithCovarianceLocal_ = *msg;

    fiducialTranslationLocal_(0,0) = fiducialPoseWithCovarianceLocal_.pose.position.x;
    fiducialTranslationLocal_(1,0) = fiducialPoseWithCovarianceLocal_.pose.position.y;
    fiducialTranslationLocal_(2,0) = fiducialPoseWithCovarianceLocal_.pose.position.z;

    Eigen::Quaterniond fiducialRotationLocal(fiducialPoseWithCovarianceLocal_.pose.orientation.w,
                                              fiducialPoseWithCovarianceLocal_.pose.orientation.x,
                                              fiducialPoseWithCovarianceLocal_.pose.orientation.y,
                                              fiducialPoseWithCovarianceLocal_.pose.orientation.z);
    fiducialRotationLocal_ = fiducialRotationLocal;                                           
    
}
