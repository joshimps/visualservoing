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

Eigen::MatrixXd Robot::getEndEffectorTransform(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransforms_.at(jointStates_.position.size());
}

Eigen::MatrixXd Robot::getJointTransform(int i){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransforms_.at(i);
}

Eigen::MatrixXd Robot::getJacobian(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jacobian_;
}


///////////////////////////////////////////////////////////////////////////////////////////
// Calculations
/////////////////////////////////////////////////////////////////////////////////////////

void Robot::calculateJointTransforms(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    Eigen::MatrixXd tRz(4,4);
    Eigen::MatrixXd tz(4,4);
    Eigen::MatrixXd tx(4,4);
    Eigen::MatrixXd tRx(4,4);

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

void Robot::calculateJointTransformsToBase(){

    Eigen::MatrixXd jointTransformToBase = jointTransforms_.at(0);

    for(int i = 1; i < jointTransforms_.size(); i++){
        
        for(int j = 1; j < i; j++){
            jointTransformToBase = jointTransformToBase * jointTransforms_.at(j); 
        }

        jointTransformsToBase_.at(i) = jointTransformToBase;
    }
}

// Please please please this needs unit testing to ensure its right
void Robot::calculateJacobian(){
    Eigen::Vector3d unitVector;
    Eigen::MatrixXd rotationMatrixIto0(3,3);
    Eigen::Vector3d translationMatrixIto0;
    Eigen::Vector3d translationMatrixNto0;
    Eigen::MatrixXd jacobianLinearVelocityComponent;

    //Lets fill in each column of the jacobian
    //Each column can be represented by the formula
    //For a robot with N DOF where I is the jacobian column index
    //rotationMatrixITo0 x unitVector x (translationMatrixN - translationMatrix(I)) }->First three rows
    //unitVector }->Last three rows
    
    //The unit vector is 0;0;1 as with DH params we always rotate about z axis
    unitVector(0,0) = 0; 
    unitVector(1,0) = 0;
    unitVector(2,0) = 1;  

    //Get the transformations of the joints with respect to the base
    calculateJointTransformsToBase();

    //The translationMatrixNto0 will be the translation part of the transformation matrix N to 0
    translationMatrixNto0(0,0) = jointTransformsToBase_.at(jointTransformsToBase_.size())(0,3);
    translationMatrixNto0(1,0) = jointTransformsToBase_.at(jointTransformsToBase_.size())(1,3);
    translationMatrixNto0(2,0) = jointTransformsToBase_.at(jointTransformsToBase_.size())(2,3);

    for(int i = -1; i < jointTransforms_.size()-1; i++){
        //Get the rotation matrix from i to 0 and the translation matrix from i to 0
        
        if(i == -1){
            //Build the matrices rotationMatrixIto0 and translationMatrixIto0
            translationMatrixIto0(0,0) = baseTransform_(0,3);
            translationMatrixIto0(1,0) = baseTransform_(1,3);
            translationMatrixIto0(2,0) = baseTransform_(2,3);

            //Row 1
            rotationMatrixIto0(0,0) = baseTransform_(0,0);
            rotationMatrixIto0(0,1) = baseTransform_(0,1);
            rotationMatrixIto0(0,2) = baseTransform_(0,2);

            //Row 2
            rotationMatrixIto0(1,0) = baseTransform_(1,0);
            rotationMatrixIto0(1,1) = baseTransform_(1,2);
            rotationMatrixIto0(1,2) = baseTransform_(1,2);

            //Row 3
            rotationMatrixIto0(2,0) = baseTransform_(2,0);
            rotationMatrixIto0(2,1) = baseTransform_(2,1);
            rotationMatrixIto0(2,2) = baseTransform_(2,2);
        }
        else{
            //Build the matrices rotationMatrixIto0 and translationMatrixIto0
            translationMatrixIto0(0,0) = jointTransformsToBase_.at(i)(0,3);
            translationMatrixIto0(1,0) = jointTransformsToBase_.at(i)(1,3);
            translationMatrixIto0(2,0) = jointTransformsToBase_.at(i)(2,3);

            //Row 1
            rotationMatrixIto0(0,0) = jointTransformsToBase_.at(i)(0,0);
            rotationMatrixIto0(0,1) = jointTransformsToBase_.at(i)(0,1);
            rotationMatrixIto0(0,2) = jointTransformsToBase_.at(i)(0,2);
            
            //Row 2
            rotationMatrixIto0(1,0) = jointTransformsToBase_.at(i)(1,0);
            rotationMatrixIto0(1,1) = jointTransformsToBase_.at(i)(1,2);
            rotationMatrixIto0(1,2) = jointTransformsToBase_.at(i)(1,2);
            //Row 3
            rotationMatrixIto0(2,0) = jointTransformsToBase_.at(i)(2,0);
            rotationMatrixIto0(2,1) = jointTransformsToBase_.at(i)(2,1);
            rotationMatrixIto0(2,2) = jointTransformsToBase_.at(i)(2,2);
        }

        //Create the coiumn in the jacobian matrix
        jacobianLinearVelocityComponent = rotationMatrixIto0 * unitVector.cross((translationMatrixNto0 - translationMatrixIto0));
        //Row 1
        jacobian_(0,i) = jacobianLinearVelocityComponent(0,0);
        //Row 2
        jacobian_(1,i) = jacobianLinearVelocityComponent(1,0);
        //Row 3
        jacobian_(2,i) = jacobianLinearVelocityComponent(2,0);
        //Row 4
        jacobian_(3,i) = unitVector(0,0);
        //Row 5
        jacobian_(4,i) = unitVector(1,0);
        //Row 6
        jacobian_(5,i) = unitVector(2,0);
    }

}
