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
    fiducialNewPose_ = nh_.advertise<geometry_msgs::Pose>("visual_servoing/fiducial_pose", 10, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

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
    ROS_DEBUG_STREAM("MOVING ROBOT \n");
    Eigen::VectorXd jointVelocities(robot_->getNumberOfJoints());

    double jointVelocitySquaredSum;
    //Keep going until the norm of the error is greater than the threshold
    //Using Position Based servoing as explained here
    //https://canvas.uts.edu.au/courses/27375/pages/2-position-based-visual-servoing-pbvs?module_item_id=1290599

    std_msgs::Float64MultiArray msg;
    
    
    jointVelocitySquaredSum = 0;
    //Calculate the jacobian of the current pose 
    
    //The joint velocity is the jacobian multiplied by the error 
    jointVelocities = robot_->getJacobian().completeOrthogonalDecomposition().pseudoInverse() * endEffectorVelocity_;
    
    //Publish the joint velocities to the robot here
    for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
        msg.data.push_back(jointVelocities(i,0));
        ROS_DEBUG_STREAM("JOINT " << i+1 << " VELOCITY: \n" << jointVelocities(i,0));
        jointVelocitySquaredSum = jointVelocitySquaredSum + jointVelocities(i,0);
    }
    euclidianNorm_ = sqrt(jointVelocitySquaredSum);
    ROS_INFO_STREAM("EUCLIDIAN ERROR \n" << euclidianNorm_);
        
    //jointVelocityPub_.publish(msg);
    ROS_DEBUG_STREAM("PUBLISHED JOINT VELOCITY \n" << jointVelocities);
}

void RobotController::stallRobot(){
    ROS_DEBUG_STREAM("STALLING ROBOT \n");
    std_msgs::Float64MultiArray msg;
    Eigen::VectorXd jointVelocities(robot_->getNumberOfJoints());

    double jointVelocitySquaredSum;

    
    jointVelocitySquaredSum = 0;
    //Calculate the jacobian of the current pose 
    //The joint velocity is the jacobian multiplied by the error 
    jointVelocities = robot_->getJacobian().completeOrthogonalDecomposition().pseudoInverse() * endEffectorVelocity_;
    //Publish the joint velocities to the robot here
    for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
        msg.data.push_back(0);
        jointVelocitySquaredSum = jointVelocitySquaredSum + jointVelocities(i,0);
    }
    euclidianNorm_ = sqrt(jointVelocitySquaredSum);
    
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
    endEffectorVelocity_(5,0) = gain_ * (fiducialRotationLocal_.z() - M_PI/2);

    msg.data.push_back(endEffectorVelocity_(0,0));
    msg.data.push_back(endEffectorVelocity_(1,0));
    msg.data.push_back(endEffectorVelocity_(2,0));
    msg.data.push_back(endEffectorVelocity_(3,0));
    msg.data.push_back(endEffectorVelocity_(4,0));
    msg.data.push_back(endEffectorVelocity_(5,0));

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
    
    geometry_msgs::PoseStamped fiducialPoseStampedLocal_ = *msg;

    //These fiducial positions need to be updated as the camera has a different coord system to the end effector

    Eigen::Quaterniond fiducialRotationWrong(fiducialPoseStampedLocal_.pose.orientation.w,
                                              fiducialPoseStampedLocal_.pose.orientation.x,
                                              fiducialPoseStampedLocal_.pose.orientation.y,
                                              fiducialPoseStampedLocal_.pose.orientation.z);

    Eigen::Matrix3d rotationMatrix = fiducialRotationWrong.normalized().toRotationMatrix();
    
    Eigen::Matrix4d transformationMatrix;
    transformationMatrix(0,0) = rotationMatrix(0,0);
    transformationMatrix(0,1) = rotationMatrix(0,1);
    transformationMatrix(0,2) = rotationMatrix(0,2);
    transformationMatrix(0,3) = fiducialPoseStampedLocal_.pose.position.x;

    transformationMatrix(1,0) = rotationMatrix(1,0);
    transformationMatrix(1,1) = rotationMatrix(1,1);
    transformationMatrix(1,2) = rotationMatrix(1,2);
    transformationMatrix(1,3) = fiducialPoseStampedLocal_.pose.position.y;

    transformationMatrix(2,0) = rotationMatrix(2,0);
    transformationMatrix(2,1) = rotationMatrix(2,1);
    transformationMatrix(2,2) = rotationMatrix(2,2);
    transformationMatrix(2,3) = fiducialPoseStampedLocal_.pose.position.z;

    transformationMatrix(3,0) = 0;
    transformationMatrix(3,1) = 0;
    transformationMatrix(3,2) = 0;
    transformationMatrix(3,3) = 1;

  
    ROS_INFO_STREAM(transformationMatrix);

    transformationMatrix = transformationMatrix;

    Eigen::Matrix3d newRotationMatrix;

    newRotationMatrix(0,0) = transformationMatrix(0,0);
    newRotationMatrix(0,1) = transformationMatrix(0,1);
    newRotationMatrix(0,2) = transformationMatrix(0,2);

    newRotationMatrix(1,0) = transformationMatrix(1,0);
    newRotationMatrix(1,1) = transformationMatrix(1,1);
    newRotationMatrix(1,2) = transformationMatrix(1,2);

    newRotationMatrix(2,0) = transformationMatrix(2,0);
    newRotationMatrix(2,1) = transformationMatrix(2,1);
    newRotationMatrix(2,2) = transformationMatrix(2,2);

    Eigen::Quaterniond newQuaternion(newRotationMatrix);

    fiducialTranslationLocal_(0,0) = transformationMatrix(0,3);
    fiducialTranslationLocal_(1,0) = transformationMatrix(1,3);
    fiducialTranslationLocal_(2,0) = transformationMatrix(2,3);

    ROS_INFO_STREAM(transformationMatrix);
    
    fiducialRotationLocal_ = newQuaternion;     

    calculateEndEffectorVelocity();
    robot_->calculateJointTransforms();
    robot_->calculateJointTransformsToBase();
    robot_->calculateJacobian(); 

    geometry_msgs::Pose pose;

    pose.orientation.w = newQuaternion.w();
    pose.orientation.x = newQuaternion.x();
    pose.orientation.y = newQuaternion.y();
    pose.orientation.w = newQuaternion.z();
    
    pose.position.x = transformationMatrix(0,3);
    pose.position.y = transformationMatrix(1,3);
    pose.position.z = transformationMatrix(2,3);

    fiducialNewPose_.publish(pose);
    
    if(euclidianNorm_ > errorThreshold_){
        //moveRobot();     
    }
    else{
        //stallRobot();
    }
    
}

void RobotController::clockCallback(const rosgraph_msgs::ClockConstPtr &msg){
    const rosgraph_msgs::Clock clock = *msg;
    std_msgs::Float64MultiArray jointMsg;
    //If gretaer than 1 second since fiducial publish
    if(clock.clock.sec - timeAtFiducialPublish_.sec > 1 && recievedFiducial_){
        ROS_ERROR_STREAM("LOST SIGHT OF FIDUCIAL");
        stallRobot();
        ros::shutdown();
    }
}