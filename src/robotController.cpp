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
    jointVelocities =  robot_->getPseudoInverseJacobianInEndEffectorFrame() * endEffectorVelocity_;
    //Publish the joint velocities to the robot here
    for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
        msg.data.push_back(jointVelocities(i,0));
    }

    euclidianNorm_ = endEffectorVelocity_.norm();
    ROS_INFO_STREAM("EUCLIDIAN ERROR \n" << euclidianNorm_);
    jointVelocityPub_.publish(msg);
    ROS_DEBUG_STREAM("PUBLISHED JOINT VELOCITY \n" << jointVelocities);
}

void RobotController::stallRobot(){
    ROS_INFO_STREAM("STALLING ROBOT \n");
    std_msgs::Float64MultiArray msg;
    Eigen::VectorXd jointVelocities(robot_->getNumberOfJoints());
   
    jointVelocities =  robot_->getPseudoInverseJacobianInEndEffectorFrame() * endEffectorVelocity_;
    //Publish the joint velocities to the robot here
    for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
        msg.data.push_back(0);
    }
    euclidianNorm_ = endEffectorVelocity_.norm();
    ROS_INFO_STREAM("EUCLIDIAN ERROR \n" << euclidianNorm_);
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
    robot_->calculateJacobianInWorldFrame(); 
    robot_->calculateJacobianInEndEffectorFrame(); 
    robot_->calculateMeasureOfManipulabilityInEndEffector();
    robot_->calculatePseudoInverseJacobianInEndEffectorFrame();
    
    geometry_msgs::PoseStamped fiducialPoseStampedGlobal_ = *msg;

    //These fiducial positions need to be updated as the camera has a different coord system to the end effector

    Eigen::Matrix4d fiducialTransformGlobal;
    Eigen::Quaterniond fiducialQuaternionGlobal;
    Eigen::Matrix3d fiducialRotationMatrixGlobal;
    Eigen::Matrix4d fiducialTransformGlobalAdjusted;
    Eigen::Matrix4d fiducialTransformEndEffector;
    Eigen::Matrix4d fiducialTransformEndEffectorAdjusted;
    Eigen::Matrix4d fiducialError;
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
    fiducialTransformGlobal(0,3) = fiducialPoseStampedGlobal_.pose.position.x + robot_->getBaseTransform()(0,3);

    fiducialTransformGlobal(1,0) = fiducialRotationMatrixGlobal(1,0);
    fiducialTransformGlobal(1,1) = fiducialRotationMatrixGlobal(1,1);
    fiducialTransformGlobal(1,2) = fiducialRotationMatrixGlobal(1,2);
    fiducialTransformGlobal(1,3) = fiducialPoseStampedGlobal_.pose.position.y + robot_->getBaseTransform()(1,3);

    fiducialTransformGlobal(2,0) = fiducialRotationMatrixGlobal(2,0);
    fiducialTransformGlobal(2,1) = fiducialRotationMatrixGlobal(2,1);
    fiducialTransformGlobal(2,2) = fiducialRotationMatrixGlobal(2,2);
    fiducialTransformGlobal(2,3) = fiducialPoseStampedGlobal_.pose.position.z + robot_->getBaseTransform()(2,3);

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

    ROS_INFO_STREAM("FIDUCIAL GLOBAL");
    ROS_INFO_STREAM(fiducialTransformGlobalAdjusted);

    fiducialError = (robot_->getJointTransformToWorld(5)).inverse() * fiducialTransformGlobalAdjusted;
    fiducialError(2,3) = fiducialError(2,3) - 0.3;
    
    //TEST
    fiducialRotationMatrixEndEffector(0,0) = fiducialError(0,0);
    fiducialRotationMatrixEndEffector(0,1) = fiducialError(0,1);
    fiducialRotationMatrixEndEffector(0,2) = fiducialError(0,2);
    fiducialTranslationEndEffector(0,0) = fiducialError(0,3);

    fiducialRotationMatrixEndEffector(1,0) = fiducialError(1,0);
    fiducialRotationMatrixEndEffector(1,1) = fiducialError(1,1);
    fiducialRotationMatrixEndEffector(1,2) = fiducialError(1,2);
    fiducialTranslationEndEffector(1,0) = fiducialError(1,3);

    fiducialRotationMatrixEndEffector(2,0) = fiducialError(2,0);
    fiducialRotationMatrixEndEffector(2,1) = fiducialError(2,1);
    fiducialRotationMatrixEndEffector(2,2) = fiducialError(2,2);
    fiducialTranslationEndEffector(2,0) = fiducialError(2,3);

    ROS_INFO_STREAM("FIDUCIAL ERROR");
    ROS_INFO_STREAM(fiducialError);

    fiducialRotationLocal_ = fiducialRotationMatrixEndEffector;
    fiducialTranslationLocal_ = fiducialTranslationEndEffector;

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
        ros::shutdown();
    }
}