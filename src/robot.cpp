#include "robot.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////


Robot::Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha){
    nh_ = nh;
    jointStateSub_ = nh_.subscribe("/joint_states", 3, &Robot::jointStateCallBack, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    d_ = d;
    a_ = a;
    alpha_ = alpha;
    jacobian_.resize(6,d.size());
    numberOfJoints_ = d_.size();

    //Row 1
    baseTransform_(0,0) = 1;
    baseTransform_(0,1) = 0;
    baseTransform_(0,2) = 0;
    baseTransform_(0,3) = 0;

    //Row 2
    baseTransform_(1,0) = 0;
    baseTransform_(1,1) = 1;
    baseTransform_(1,2) = 0;
    baseTransform_(1,3) = 0;

    //Row 3
    baseTransform_(2,0) = 0;
    baseTransform_(2,1) = 0;
    baseTransform_(2,2) = 1;
    baseTransform_(2,3) = 0;

    //Row 4
    baseTransform_(3,0) = 0;
    baseTransform_(3,1) = 0;
    baseTransform_(3,2) = 0;
    baseTransform_(3,3) = 1;
    
    
}

///////////////////////////////////////////////////////////
// Callbacks
//////////////////////////////////////////////////////////

void Robot::jointStateCallBack(const sensor_msgs::JointStateConstPtr &msg){
    ROS_DEBUG_STREAM("RECIEVED JOINT TRANSFORM");
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    jointStates_ = *msg;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Getters
/////////////////////////////////////////////////////////////////////////////////////////

sensor_msgs::JointState Robot::getJointState(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointStates_;
}

Eigen::MatrixXd Robot::getEndEffectorTransform(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransforms_.at(numberOfJoints_-1);
}

Eigen::MatrixXd Robot::getJointTransform(int i){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    
    return jointTransforms_.at(i);
}

Eigen::MatrixXd Robot::getJointTransformToBase(int i){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    
    return jointTransformsToBase_.at(i);
}

Eigen::MatrixXd Robot::getJacobian(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jacobian_;
}

int Robot::getNumberOfJoints(){
    return numberOfJoints_;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Setters
/////////////////////////////////////////////////////////////////////////////////////////

void Robot::setTheta(std::vector<double> theta){
    theta_ = theta;
    sensor_msgs::JointState jointStates;
    jointStates_.position = theta;
}


///////////////////////////////////////////////////////////////////////////////////////////
// Calculations
/////////////////////////////////////////////////////////////////////////////////////////

void Robot::calculateJointTransforms(){
    jointTransforms_.clear();

    Eigen::Matrix4d tRz;
    Eigen::Matrix4d tz;
    Eigen::Matrix4d tx;
    Eigen::Matrix4d tRx;

    ROS_DEBUG_STREAM("JOINT TRANSFORMS");
    for(int i = 0; i < d_.size(); i++){
        //tRz
        
        //Row 1
        tRz(0,0) = cos(jointStates_.position.at(i));
        tRz(0,1) = -sin(jointStates_.position.at(i));
        tRz(0,2) = 0;
        tRz(0,3) = 0;
        
        //Row 2
        tRz(1,0) = sin(jointStates_.position.at(i));
        tRz(1,1) = cos(jointStates_.position.at(i));
        tRz(1,2) = 0;
        tRz(1,3) = 0;

        //Row 3
        tRz(2,0) = 0;
        tRz(2,1) = 0;
        tRz(2,2) = 1;
        tRz(2,3) = 0;

        //Row 4
        tRz(3,0) = 0;
        tRz(3,1) = 0;
        tRz(3,2) = 0;
        tRz(3,3) = 1;

        //tz

        //Row 1
        tz(0,0) = 1;
        tz(0,1) = 0;
        tz(0,2) = 0;
        tz(0,3) = 0;

        //Row 2
        tz(1,0) = 0;
        tz(1,1) = 1;
        tz(1,2) = 0;
        tz(1,3) = 0;

        //Row 3
        tz(2,0) = 0;
        tz(2,1) = 0;
        tz(2,2) = 1;
        tz(2,3) = d_.at(i);

        //Row 4
        tz(3,0) = 0;
        tz(3,1) = 0;
        tz(3,2) = 0;
        tz(3,3) = 1;

        //tx

        //Row 1
        tx(0,0) = 1;
        tx(0,1) = 0;
        tx(0,2) = 0;
        tx(0,3) = a_.at(i);

        //Row 2
        tx(1,0) = 0;
        tx(1,1) = 1;
        tx(1,2) = 0;
        tx(1,3) = 0;

        //Row 3
        tx(2,0) = 0;
        tx(2,1) = 0;
        tx(2,2) = 1;
        tx(2,3) = 0;

        //Row 4
        tx(3,0) = 0;
        tx(3,1) = 0;
        tx(3,2) = 0;
        tx(3,3) = 1;

        //tRx

        //Row 1
        tRx(0,0) = 1;
        tRx(0,1) = 0;
        tRx(0,2) = 0;
        tRx(0,3) = 0;

        //Row 2
        tRx(1,0) = 0;
        tRx(1,1) = cos(alpha_.at(i));
        tRx(1,2) = -sin(alpha_.at(i));
        tRx(1,3) = 0;

        //Row 3
        tRx(2,0) = 0;
        tRx(2,1) = sin(alpha_.at(i));
        tRx(2,2) = cos(alpha_.at(i));
        tRx(2,3) = 0;

        //Row 4
        tRx(3,0) = 0;
        tRx(3,1) = 0;
        tRx(3,2) = 0;
        tRx(3,3) = 1;
        
        jointTransforms_.push_back(tRz * tz * tx * tRx);
        
        ROS_DEBUG_STREAM("\n" << tRz * tz * tx * tRx);

    }   
}

void Robot::calculateJointTransformsToBase(){
    jointTransformsToBase_.clear();
    Eigen::MatrixXd jointTransformToBase = jointTransforms_.at(0);
    jointTransformsToBase_.push_back(jointTransformToBase);
    ROS_DEBUG_STREAM("JOINT TO BASE TRANSFORMS");
    for(int i = 1; i < jointTransforms_.size(); i++){
        
        jointTransformToBase = jointTransformToBase * jointTransforms_.at(i); 
        ROS_DEBUG_STREAM("\n" << jointTransformToBase);
        jointTransformsToBase_.push_back(jointTransformToBase);
    }
}

// Please please please this needs unit testing to ensure its right
void Robot::calculateJacobian(){
    
    Eigen::Vector3d unitVector;
    Eigen::Matrix3d rotationMatrixItoB;
    Eigen::Vector3d translationMatrixItoB;
    Eigen::Vector3d translationMatrixNtoB;
    Eigen::Vector3d jacobianLinearVelocityComponent;
    Eigen::Vector3d jacobianRotationalVelocityComponent;

    //Lets fill in each column of the jacobian
    //Each column can be represented by the formula
    //For a robot with N DOF where I is the jacobian column index and B is the base transform
    //If I was 0 then the transform would be from BtoB or 0to0
    //rotationMatrixIToB x unitVector x (translationMatrixNtoB - translationMatrix(ItoB) }->First three rows
    //unitVector }->Last three rows
    
    //The unit vector is 0;0;1 as with DH params we always rotate about z axis
    unitVector(0,0) = 0; 
    unitVector(1,0) = 0;
    unitVector(2,0) = 1;  

    //The translationMatrixNto0 will be the translation part of the transformation matrix N to 0
    translationMatrixNtoB(0,0) = jointTransformsToBase_.at(jointTransformsToBase_.size()-1)(0,3);
    translationMatrixNtoB(1,0) = jointTransformsToBase_.at(jointTransformsToBase_.size()-1)(1,3);
    translationMatrixNtoB(2,0) = jointTransformsToBase_.at(jointTransformsToBase_.size()-1)(2,3);
    
    for(int i = 0; i < jointTransforms_.size(); i++){
        //Get the rotation matrix from i to 0 and the translation matrix from i to 0
        
        if(i == 0){
            //Build the matrices rotationMatrixIto0 and translationMatrixIto0
            translationMatrixItoB(0,0) = baseTransform_(0,3);
            translationMatrixItoB(1,0) = baseTransform_(1,3);
            translationMatrixItoB(2,0) = baseTransform_(2,3);

            //Row 1
            rotationMatrixItoB(0,0) = baseTransform_(0,0);
            rotationMatrixItoB(0,1) = baseTransform_(0,1);
            rotationMatrixItoB(0,2) = baseTransform_(0,2);

            //Row 2
            rotationMatrixItoB(1,0) = baseTransform_(1,0);
            rotationMatrixItoB(1,1) = baseTransform_(1,1);
            rotationMatrixItoB(1,2) = baseTransform_(1,2);

            //Row 3
            rotationMatrixItoB(2,0) = baseTransform_(2,0);
            rotationMatrixItoB(2,1) = baseTransform_(2,1);
            rotationMatrixItoB(2,2) = baseTransform_(2,2);
        }
        else{
            //Build the matrices rotationMatrixIto0 and translationMatrixIto0
            translationMatrixItoB(0,0) = jointTransformsToBase_.at(i-1)(0,3);
            translationMatrixItoB(1,0) = jointTransformsToBase_.at(i-1)(1,3);
            translationMatrixItoB(2,0) = jointTransformsToBase_.at(i-1)(2,3);
            
            //Row 1
            rotationMatrixItoB(0,0) = jointTransformsToBase_.at(i-1)(0,0);
            rotationMatrixItoB(0,1) = jointTransformsToBase_.at(i-1)(0,1);
            rotationMatrixItoB(0,2) = jointTransformsToBase_.at(i-1)(0,2);
            
            //Row 2
            rotationMatrixItoB(1,0) = jointTransformsToBase_.at(i-1)(1,0);
            rotationMatrixItoB(1,1) = jointTransformsToBase_.at(i-1)(1,1);
            rotationMatrixItoB(1,2) = jointTransformsToBase_.at(i-1)(1,2);
            //Row 3
            rotationMatrixItoB(2,0) = jointTransformsToBase_.at(i-1)(2,0);
            rotationMatrixItoB(2,1) = jointTransformsToBase_.at(i-1)(2,1);
            rotationMatrixItoB(2,2) = jointTransformsToBase_.at(i-1)(2,2);
        }
        
        //Create the coiumn in the jacobian matrix
        jacobianLinearVelocityComponent = (rotationMatrixItoB * unitVector).cross((translationMatrixNtoB - translationMatrixItoB));
        jacobianRotationalVelocityComponent = rotationMatrixItoB * unitVector;


        //Row 1
        jacobian_(0,i) = jacobianLinearVelocityComponent(0,0);
        //Row 2
        jacobian_(1,i) = jacobianLinearVelocityComponent(1,0);
        //Row 3
        jacobian_(2,i) = jacobianLinearVelocityComponent(2,0);
        //Row 4
        jacobian_(3,i) = jacobianRotationalVelocityComponent(0,0);
        //Row 5
        jacobian_(4,i) = jacobianRotationalVelocityComponent(1,0);
        //Row 6
        jacobian_(5,i) = jacobianRotationalVelocityComponent(2,0);
    }
    ROS_DEBUG_STREAM("JACOBIAN");
    ROS_DEBUG_STREAM("\n" << jacobian_);
}
