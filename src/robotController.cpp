#include "robotController.h"


///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////

RobotController::RobotController(ros::NodeHandle nh, Robot* robot, double gain, double errorThreshold){
    nh_ = nh;
    robot_ = robot;
    gain_ = gain;
    errorThreshold_ = errorThreshold;
    fiducialPositionSub_ = nh_.subscribe("/aruco_single/pose", 10, &RobotController::fiducialPositionCallBack, this);
    jointVelocityPub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_vel_controller/command", 10, false);
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
    
    Eigen::VectorXd endEffectorVelocity(6);
    Eigen::VectorXd jointVelocities(robot_->getNumberOfJoints());

    double jointVelocitySquaredSum;
    volatile double euclidianNorm = 100;
    //Keep going until the norm of the error is greater than the threshold
    //Using Position Based servoing as explained here
    //https://canvas.uts.edu.au/courses/27375/pages/2-position-based-visual-servoing-pbvs?module_item_id=1290599

    while(euclidianNorm > errorThreshold_){
        ROS_INFO_STREAM("MOVING ROBOT");
        jointVelocitySquaredSum = 0;
        //Calculate our end effector velocity from the positional and rotational error
        endEffectorVelocity(0,0) = gain_ * fiducialTranslationLocal_(0,0);
        endEffectorVelocity(1,0) = gain_ * fiducialTranslationLocal_(1,0);
        endEffectorVelocity(2,0) = gain_ * fiducialTranslationLocal_(2,0);
        endEffectorVelocity(3,0) = gain_ * fiducialRotationLocal_.x();
        endEffectorVelocity(4,0) = gain_ * fiducialRotationLocal_.y();
        endEffectorVelocity(5,0) = gain_ * fiducialRotationLocal_.z();
        //Calculate the jacobian of the current pose 
        robot_->calculateJointTransforms();
        robot_->calculateJointTransformsToBase();
        robot_->calculateJacobian();
        //The joint velocity is the jacobian multiplied by the error 
        jointVelocities = robot_->getJacobian().completeOrthogonalDecomposition().pseudoInverse() * endEffectorVelocity;
        ROS_INFO_STREAM(jointVelocities);
        //Publish the joint velocities to the robot here
        std_msgs::Float64MultiArray msg;
        std::stringstream ss;
        for(int i = 0; i < (robot_->getNumberOfJoints()); i++){
            msg.data.push_back(jointVelocities(i,0));
            jointVelocitySquaredSum = jointVelocitySquaredSum + jointVelocities.coeff(i,0);
        }

        jointVelocityPub_.publish(msg);
        euclidianNorm = sqrt(jointVelocitySquaredSum);
        ROS_INFO_STREAM("PUBLISHED");
    }

    ROS_INFO_STREAM("END EFFECTOR AT \n" << robot_->getEndEffectorTransform());
}

///////////////////////////////////////////////////////////
// Callbacks and Services
//////////////////////////////////////////////////////////

void RobotController::fiducialPositionCallBack(const geometry_msgs::PoseStampedPtr &msg){
    ROS_INFO_STREAM("FIDUCIAL POSITION RECIEVED");
    std::unique_lock<std::mutex> lck(fiducialPoseMutex_);

    geometry_msgs::PoseStamped fiducialPoseStampedLocal_ = *msg;
    
    fiducialTranslationLocal_(0,0) = fiducialPoseStampedLocal_.pose.position.x;
    fiducialTranslationLocal_(1,0) = fiducialPoseStampedLocal_.pose.position.y;
    fiducialTranslationLocal_(2,0) = fiducialPoseStampedLocal_.pose.position.z;

    Eigen::Quaterniond fiducialRotationLocal(fiducialPoseStampedLocal_.pose.orientation.w,
                                              fiducialPoseStampedLocal_.pose.orientation.x,
                                              fiducialPoseStampedLocal_.pose.orientation.y,
                                              fiducialPoseStampedLocal_.pose.orientation.z);
    fiducialRotationLocal_ = fiducialRotationLocal;      
    
    
    moveRobot();     
}
