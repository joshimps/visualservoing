#include "robotController.h"


///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////

RobotController::RobotController(ros::NodeHandle nh, Robot* robot, double gain, double errorThreshold){
    nh_ = nh;
    robot_ = robot;
    gain_ = gain;
    errorThreshold_ = errorThreshold;
    euclidianNorm_ = errorThreshold + 1;
    recievedFiducial_ = false;
    endEffectorVelocity_.resize(6);
    endEffectorVelocity_(0,0) = 0;
    endEffectorVelocity_(1,0) = 0;
    endEffectorVelocity_(2,0) = 0;
    endEffectorVelocity_(3,0) = 0;
    endEffectorVelocity_(4,0) = 0;
    endEffectorVelocity_(5,0) = 0;
    fiducialPositionSub_ = nh_.subscribe("/aruco_single/pose", 10, &RobotController::fiducialPositionCallBack, this);
    clockSub_ = nh_.subscribe("/clock", 10, &RobotController::clockCallback, this);
    jointVelocityPub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_vel_controller/command", 10, false);
    endEffectorVelocityPub_ = nh_.advertise<std_msgs::Float64MultiArray>("visual_servoing/end_effector_velocity", 10, false);
    fiducialNewPose_ = nh_.advertise<geometry_msgs::PoseStamped>("visual_servoing/fiducial_pose", 10, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

}

///////////////////////////////////////////////////////////////////////////////////////////
// Getters
/////////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd RobotController::getEndEffectorVelocity(){
    return endEffectorVelocity_;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Setters
/////////////////////////////////////////////////////////////////////////////////////////

void RobotController::setFiducialPostition(Eigen::MatrixXd fiducialTranslationLocal , Eigen::Quaterniond fiducialRotationLocal){
    fiducialTranslationLocal_ = fiducialTranslationLocal;
    fiducialRotationLocal_ = fiducialRotationLocal;
}

///////////////////////////////////////////////////////////
// Actions
//////////////////////////////////////////////////////////

void RobotController::moveRobot(){
    ROS_INFO_STREAM("MOVING ROBOT \n");
    std_msgs::Float64MultiArray msg;
    Eigen::VectorXd jointVelocities(robot_->getNumberOfJoints());

    //Calculate the jacobian of the current pose 
    //The joint velocity is the jacobian multiplied by the error 
    ROS_INFO_STREAM("STALLING ROBOT \n");
    jointVelocities =  robot_->getPseudoInverseJacobian() * endEffectorVelocity_;
    //Publish the joint velocities to the robot here
    for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
        msg.data.push_back(jointVelocities(i,0));
    }

    euclidianNorm_ = endEffectorVelocity_.norm();
    ROS_INFO_STREAM("ERROR \n" << euclidianNorm_);
    jointVelocityPub_.publish(msg);
    ROS_DEBUG_STREAM("PUBLISHED JOINT VELOCITY \n" << jointVelocities);
}

void RobotController::stallRobot(){
    ROS_INFO_STREAM("STALLING ROBOT \n");
    std_msgs::Float64MultiArray msg;
    Eigen::VectorXd jointVelocities(robot_->getNumberOfJoints());
   
    jointVelocities =  robot_->getPseudoInverseJacobian() * endEffectorVelocity_;
    //Publish the joint velocities to the robot here
    for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
        msg.data.push_back(0);
    }
    euclidianNorm_ = endEffectorVelocity_.norm();
    ROS_INFO_STREAM("ERROR \n" << euclidianNorm_);
    jointVelocityPub_.publish(msg);
    ROS_DEBUG_STREAM("PUBLISHED JOINT VELOCITY \n" << jointVelocities);
}

void RobotController::calculateEndEffectorVelocity(){
    std_msgs::Float64MultiArray msg;

    endEffectorVelocity_(0,0) = gain_ * fiducialTranslationLocal_(0,0);
    endEffectorVelocity_(1,0) = gain_ * fiducialTranslationLocal_(1,0);
    endEffectorVelocity_(2,0) = gain_ * fiducialTranslationLocal_(2,0);
    endEffectorVelocity_(3,0) = gain_ * fiducialRotationLocal_.x();
    endEffectorVelocity_(4,0) = gain_ * fiducialRotationLocal_.y();
    endEffectorVelocity_(5,0) = gain_ * fiducialRotationLocal_.z();

    msg.data.push_back(endEffectorVelocity_(0,0));
    msg.data.push_back(endEffectorVelocity_(1,0));
    msg.data.push_back(endEffectorVelocity_(2,0));
    msg.data.push_back(endEffectorVelocity_(3,0));
    msg.data.push_back(endEffectorVelocity_(4,0));
    msg.data.push_back(endEffectorVelocity_(5,0));

     ROS_DEBUG_STREAM("END EFFECTOR VELOCITIES \n" << endEffectorVelocity_);

    endEffectorVelocityPub_.publish(msg);

}

///////////////////////////////////////////////////////////
// Callbacks and Services
//////////////////////////////////////////////////////////

void RobotController::fiducialPositionCallBack(const geometry_msgs::PoseStampedPtr &msg){
    ROS_DEBUG_STREAM("FIDUCIAL POSITION RECIEVED");
    recievedFiducial_ = true;
    timeAtFiducialPublish_ = ros::Time::now();
    std::unique_lock<std::mutex> lck(fiducialPoseMutex_);

    robot_->calculateJointTransforms();
    robot_->calculateJointTransformsToBase();
    robot_->calculateJointTransformsToWorld();
    robot_->calculateJacobian(); 
    
    geometry_msgs::PoseStamped fiducialPoseStampedGlobal_ = *msg;

    //These fiducial positions need to be updated as the camera has a different coord system to the end effector

    Eigen::Matrix4d fiducialTransformGlobal;
    Eigen::Quaterniond fiducialQuaternionGlobal;
    Eigen::Matrix3d fiducialRotationMatrixGlobal;
    Eigen::Matrix4d fiducialTransformGlobalAdjusted;
    Eigen::Matrix4d fiducialTransformEndEffector;
    Eigen::Matrix3d fiducialRotationMatrixEndEffector;
    Eigen::Vector3d fiducialTranslationEndEffector;

    fiducialQuaternionGlobal.w() = fiducialPoseStampedGlobal_.pose.orientation.w;
    fiducialQuaternionGlobal.x() = fiducialPoseStampedGlobal_.pose.orientation.x;
    fiducialQuaternionGlobal.y() = fiducialPoseStampedGlobal_.pose.orientation.y;
    fiducialQuaternionGlobal.z() = fiducialPoseStampedGlobal_.pose.orientation.z;

    fiducialRotationMatrixGlobal = fiducialQuaternionGlobal.normalized().toRotationMatrix();

    fiducialTransformGlobal(0,0) = fiducialRotationMatrixGlobal(0,0);
    fiducialTransformGlobal(0,1) = fiducialRotationMatrixGlobal(0,1);
    fiducialTransformGlobal(0,2) = fiducialRotationMatrixGlobal(0,2);
    fiducialTransformGlobal(0,3) = fiducialPoseStampedGlobal_.pose.position.x;

    fiducialTransformGlobal(1,0) = fiducialRotationMatrixGlobal(1,0);
    fiducialTransformGlobal(1,1) = fiducialRotationMatrixGlobal(1,1);
    fiducialTransformGlobal(1,2) = fiducialRotationMatrixGlobal(1,2);
    fiducialTransformGlobal(1,3) = fiducialPoseStampedGlobal_.pose.position.y;

    fiducialTransformGlobal(2,0) = fiducialRotationMatrixGlobal(2,0);
    fiducialTransformGlobal(2,1) = fiducialRotationMatrixGlobal(2,1);
    fiducialTransformGlobal(2,2) = fiducialRotationMatrixGlobal(2,2);
    fiducialTransformGlobal(2,3) = fiducialPoseStampedGlobal_.pose.position.z;

    fiducialTransformGlobal(3,0) = 0;
    fiducialTransformGlobal(3,1) = 0;
    fiducialTransformGlobal(3,2) = 0;
    fiducialTransformGlobal(3,3) = 1;

    Eigen::Matrix4d adjustmentMatrix;

    adjustmentMatrix(0,0) = 1;
    adjustmentMatrix(0,1) = 0;
    adjustmentMatrix(0,2) = 0;
    adjustmentMatrix(0,3) = 0;

    adjustmentMatrix(1,0) = 0;
    adjustmentMatrix(1,1) = -1;
    adjustmentMatrix(1,2) = 0;
    adjustmentMatrix(1,3) = 0;

    adjustmentMatrix(2,0) = 0;
    adjustmentMatrix(2,1) = 0;
    adjustmentMatrix(2,2) = -1;
    adjustmentMatrix(2,3) = 0;

    adjustmentMatrix(3,0) = 0;
    adjustmentMatrix(3,1) = 0;
    adjustmentMatrix(3,2) = 0;
    adjustmentMatrix(3,3) = 1;

    fiducialTransformGlobalAdjusted = fiducialTransformGlobal * adjustmentMatrix;

    fiducialTransformEndEffector = fiducialTransformGlobalAdjusted * (robot_->getJointTransformToWorld(5)).inverse();

    fiducialRotationMatrixEndEffector(0,0) = fiducialTransformEndEffector(0,0);
    fiducialRotationMatrixEndEffector(0,1) = fiducialTransformEndEffector(0,1);
    fiducialRotationMatrixEndEffector(0,2) = fiducialTransformEndEffector(0,2);
    fiducialTranslationEndEffector(0,0) = fiducialTransformEndEffector(0,3);

    fiducialRotationMatrixEndEffector(1,0) = fiducialTransformEndEffector(1,0);
    fiducialRotationMatrixEndEffector(1,1) = fiducialTransformEndEffector(1,1);
    fiducialRotationMatrixEndEffector(1,2) = fiducialTransformEndEffector(1,2);
    fiducialTranslationEndEffector(1,0) = fiducialTransformEndEffector(1,3);

    fiducialRotationMatrixEndEffector(2,0) = fiducialTransformEndEffector(2,0);
    fiducialRotationMatrixEndEffector(2,1) = fiducialTransformEndEffector(2,1);
    fiducialRotationMatrixEndEffector(2,2) = fiducialTransformEndEffector(2,2);
    fiducialTranslationEndEffector(2,0) = fiducialTransformEndEffector(2,3);

    fiducialRotationLocal_ = fiducialRotationMatrixEndEffector;
    fiducialTranslationLocal_ = fiducialTranslationEndEffector;


    geometry_msgs::PoseStamped pose;

    Eigen::Matrix3d endEffectorMatrixEndEffector;
    Eigen::Vector3d endEffectorTranslation;
    Eigen::Quaterniond endEffectorQuaternion;

    endEffectorMatrixEndEffector(0,0) = robot_->getJointTransformToWorld(5)(0,0);
    endEffectorMatrixEndEffector(0,1) = robot_->getJointTransformToWorld(5)(0,1);
    endEffectorMatrixEndEffector(0,2) = robot_->getJointTransformToWorld(5)(0,2);
    endEffectorTranslation(0,0) =       robot_->getJointTransformToWorld(5)(0,3);

    endEffectorMatrixEndEffector(1,0) = robot_->getJointTransformToWorld(5)(1,0);
    endEffectorMatrixEndEffector(1,1) = robot_->getJointTransformToWorld(5)(1,1);
    endEffectorMatrixEndEffector(1,2) = robot_->getJointTransformToWorld(5)(1,2);
    endEffectorTranslation(1,0) =       robot_->getJointTransformToWorld(5)(1,3);

    endEffectorMatrixEndEffector(2,0) = robot_->getJointTransformToWorld(5)(2,0);
    endEffectorMatrixEndEffector(2,1) = robot_->getJointTransformToWorld(5)(2,1);
    endEffectorMatrixEndEffector(2,2) = robot_->getJointTransformToWorld(5)(2,2);
    endEffectorTranslation(2,0) =       robot_->getJointTransformToWorld(5)(2,3);

    endEffectorQuaternion = endEffectorMatrixEndEffector;

    pose.pose.orientation.w = endEffectorQuaternion.w();
    pose.pose.orientation.x = endEffectorQuaternion.x();
    pose.pose.orientation.y = endEffectorQuaternion.y();
    pose.pose.orientation.z = endEffectorQuaternion.z();

    pose.pose.position.x = endEffectorTranslation(0,0);
    pose.pose.position.y = endEffectorTranslation(1,0);
    pose.pose.position.z = endEffectorTranslation(2,0);

    pose.header.frame_id = fiducialPoseStampedGlobal_.header.frame_id;

    fiducialNewPose_.publish(pose);

    ROS_DEBUG_STREAM("FIDUCIAL TRANSFORM END EFFECTOR");
    ROS_DEBUG_STREAM("\n" << fiducialTransformEndEffector);
    
    calculateEndEffectorVelocity();

    if(euclidianNorm_ > errorThreshold_){
        moveRobot();     
    }
    else{
        stallRobot();
    }
    
}

void RobotController::clockCallback(const rosgraph_msgs::ClockConstPtr &msg){
    const rosgraph_msgs::Clock clock = *msg;
    std_msgs::Float64MultiArray jointMsg;
    //If gretaer than 1 second since fiducial publish
    if(clock.clock.sec - timeAtFiducialPublish_.sec > 1 && recievedFiducial_){
        ROS_ERROR_STREAM("LOST SIGHT OF FIDUCIAL");
    }
}