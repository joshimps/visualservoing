#include "robot.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////

Robot::Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha){
    nh_ = nh;
    jointStateSub_ = nh_.subscribe("/joint_states", 3, &Robot::jointStateCallBack, this);
    d_ = d;
    a_ = a;
    alpha_ = alpha;

    tf2::Vector3 baseTranslation(0,0,0);
    tf2::Matrix3x3 baseRotation(1,0,0,
                              0,1,0,
                              0,0,1);
    baseTransform_.setOrigin(baseTranslation);
    baseTransform_.setRotation(baseRotation);
}

///////////////////////////////////////////////////////////
// Callbacks
//////////////////////////////////////////////////////////

void Robot::jointStateCallBack(const sensor_msgs::JointStateConstPtr &msg){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    sensor_msgs::JointState jointStates_ = *msg;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Getters
/////////////////////////////////////////////////////////////////////////////////////////

sensor_msgs::JointState Robot::getJointState(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointStates_;
}

tf2::Transform Robot::getEndEffectorTransform(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransforms_.at(jointStates_.position.size());
}

///////////////////////////////////////////////////////////////////////////////////////////
// Calculations
/////////////////////////////////////////////////////////////////////////////////////////

void Robot::calculateJointTransforms(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    Eigen::MatrixXd tRz;
    Eigen::MatrixXd tz;
    Eigen::MatrixXd tx;
    Eigen::MatrixXd tRx;

    for(int i = 0; i < jointStates_.position.size(); i++){
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
        tz(2,2) = d_.at(i);
        tz(2,3) = 0;

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
        tx(2,2) = 0;
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
        tRx(0,3) = a_.at(i);

        //Row 2
        tRx(1,0) = 0;
        tRx(1,1) = cos(jointStates_.position.at(i));
        tRx(1,2) = -sin(jointStates_.position.at(i));
        tRx(1,3) = 0;

        //Row 3
        tRx(2,0) = 0;
        tRx(2,1) = sin(jointStates_.position.at(i));
        tRx(2,2) = cos(jointStates_.position.at(i));
        tRx(2,3) = 0;

        //Row 4
        tRx(3,0) = 0;
        tRx(3,1) = 0;
        tRx(3,2) = 0;
        tRx(3,3) = 1;

        jointTransforms_.at(i) = tRz * tz * tx * tRx;
    }   
}

void Robot::calculateJacobian(){
    Eigen::MatrixXd unitVector;
    Eigen::MatrixXd rotationMatrix;
    Eigen::MatrixXd translationMatrix;

    //Lets fill in each column of the jacobian
    //Each column can be represented by the formula
    //For a robot with N DOF where I is the column number
    //rotationMatrixN x unitVector x (translationMatrixN - translationMatrix(I-1))
    
    for(int i = 0; i < jointTransforms_.size();i++){

    }


    return jacobian;
}
