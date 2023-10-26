#include "robot.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructors
/////////////////////////////////////////////////////////////////////////////////////////


Robot::Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha, std::vector<std::string> jointNames){
    nh_ = nh;
    jointStateSub_ = nh_.subscribe("/joint_states", 3, &Robot::jointStateCallBack, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    d_ = d;
    a_ = a;
    alpha_ = alpha;
    jointNames_ = jointNames;
    jacobianInWorldFrame_.resize(6,d.size());
    jacobianInEndEffectorFrame_.resize(6,d.size());
    dampingThreshold_ = 0.001;
    numberOfJoints_ = d_.size();

    //Row 1
    baseTransform_(0,0) = -1;
    baseTransform_(0,1) = 0;
    baseTransform_(0,2) = 0;
    baseTransform_(0,3) = 0;

    //Row 2
    baseTransform_(1,0) = 0;
    baseTransform_(1,1) = -1;
    baseTransform_(1,2) = 0;
    baseTransform_(1,3) = 0;

    //Row 3
    baseTransform_(2,0) = 0;
    baseTransform_(2,1) = 0;
    baseTransform_(2,2) = 1;
    baseTransform_(2,3) = 0.69;

    //Row 4
    baseTransform_(3,0) = 0;
    baseTransform_(3,1) = 0;
    baseTransform_(3,2) = 0;
    baseTransform_(3,3) = 1;

    for(int i = 0; i < jointNames_.size(); i++){
        jointTransforms_.push_back(baseTransform_);
    }
    
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
    return jointTransformsToBase_.at(numberOfJoints_-1) * baseTransform_;
}

Eigen::MatrixXd Robot::getJointTransform(int i){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransforms_.at(i);
}

Eigen::MatrixXd Robot::getBaseTransform(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return baseTransform_;
}


Eigen::MatrixXd Robot::getJointTransformToBase(int i){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransformsToBase_.at(i);
}

Eigen::MatrixXd Robot::getJointTransformToWorld(int i){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jointTransformsToWorld_.at(i);
}

Eigen::MatrixXd Robot::getJacobianInWorldFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jacobianInWorldFrame_;
}

Eigen::MatrixXd Robot::getJacobianInEndEffectorFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jacobianInEndEffectorFrame_;
}

Eigen::MatrixXd Robot::getTransposeJacobianInWorldFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jacobianInWorldFrame_.transpose();
}

Eigen::MatrixXd Robot::getTransposeJacobianInEndEffectorFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return jacobianInEndEffectorFrame_.transpose();
}

Eigen::MatrixXd Robot::getPseudoInverseJacobianInWorldFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return pseudoInverseJacobianInWorldFrame_;
}

Eigen::MatrixXd Robot::getPseudoInverseJacobianInEndEffectorFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return pseudoInverseJacobianInEndEffectorFrame_;
}

int Robot::getNumberOfJoints(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    return numberOfJoints_;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Setters
/////////////////////////////////////////////////////////////////////////////////////////

void Robot::setTheta(std::vector<double> theta, std::vector<std::string> jointNames){
    theta_ = theta;
    sensor_msgs::JointState jointStates;
    jointStates_.position = theta;
    jointStates_.name = jointNames;
}


///////////////////////////////////////////////////////////////////////////////////////////
// Calculations
/////////////////////////////////////////////////////////////////////////////////////////

void Robot::calculateJointTransforms(){

    std::unique_lock<std::mutex> lck(jointStateMutex_);
    
    Eigen::Matrix4d tRz;
    Eigen::Matrix4d tz;
    Eigen::Matrix4d tx;
    Eigen::Matrix4d tRx;

    ROS_DEBUG_STREAM("JOINT TRANSFORMS");

    std::vector<int> jointOrder;

    if(jointStates_.name.size() != jointNames_.size()){
        ROS_ERROR_STREAM("MORE JOINTS FOUND IN MESSAGE THAN PROVIDED IN jointNames_");
    }

    for(int i = 0; i < jointNames_.size(); i++){
        for(int j = 0; j < jointStates_.name.size(); j++){
            if(jointNames_.at(i) == jointStates_.name.at(j)){
                jointOrder.push_back(j);
            }
        }
    }
    
    for(int i = 0; i < jointNames_.size(); i++){
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
        tz(2,3) = d_.at(jointOrder.at(i));

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
        tx(0,3) = a_.at(jointOrder.at(i));

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
        tRx(1,1) = cos(alpha_.at(jointOrder.at(i)));
        tRx(1,2) = -sin(alpha_.at(jointOrder.at(i)));
        tRx(1,3) = 0;

        //Row 3
        tRx(2,0) = 0;
        tRx(2,1) = sin(alpha_.at(jointOrder.at(i)));
        tRx(2,2) = cos(alpha_.at(jointOrder.at(i)));
        tRx(2,3) = 0;

        //Row 4
        tRx(3,0) = 0;
        tRx(3,1) = 0;
        tRx(3,2) = 0;
        tRx(3,3) = 1;

        jointTransforms_.at(jointOrder.at(i)) = (tRz * tz * tx * tRx);
        ROS_DEBUG_STREAM("JOINT " << jointOrder.at(i) << "\n" << tRz * tz * tx * tRx);

    }   
}

void Robot::calculateJointTransformsToBase(){
    jointTransformsToBase_.clear();
    Eigen::MatrixXd jointTransformToBase = jointTransforms_.at(0);
    jointTransformsToBase_.push_back(jointTransformToBase);

    ROS_DEBUG_STREAM("\n" << baseTransform_);
    ROS_DEBUG_STREAM("JOINT TO BASE TRANSFORMS");
    ROS_DEBUG_STREAM("\n" << jointTransformToBase);
    for(int i = 1; i < jointTransforms_.size(); i++){
        
        jointTransformToBase = jointTransformToBase * jointTransforms_.at(i); 
        ROS_DEBUG_STREAM("\n" << jointTransformToBase);
        jointTransformsToBase_.push_back(jointTransformToBase);
    }
}

void Robot::calculateJointTransformsToWorld(){
    jointTransformsToWorld_.clear();
    ROS_DEBUG_STREAM("JOINT TO WORLD TRANSFORMS");
    for(int i = 0; i < jointTransformsToBase_.size(); i++){
        Eigen::MatrixXd jointTransformToWorld = (baseTransform_ * jointTransformsToBase_.at(i));
        
        ROS_DEBUG_STREAM("\n" << jointTransformToWorld);
        jointTransformsToWorld_.push_back(jointTransformToWorld);
    }
}

void Robot::calculateJacobianInWorldFrame(){
    
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
    translationMatrixNtoB(0,0) = jointTransformsToWorld_.at(jointTransformsToWorld_.size()-1)(0,3);
    translationMatrixNtoB(1,0) = jointTransformsToWorld_.at(jointTransformsToWorld_.size()-1)(1,3);
    translationMatrixNtoB(2,0) = jointTransformsToWorld_.at(jointTransformsToWorld_.size()-1)(2,3);
    
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
            translationMatrixItoB(0,0) = jointTransformsToWorld_.at(i-1)(0,3);
            translationMatrixItoB(1,0) = jointTransformsToWorld_.at(i-1)(1,3);
            translationMatrixItoB(2,0) = jointTransformsToWorld_.at(i-1)(2,3);
            
            //Row 1
            rotationMatrixItoB(0,0) = jointTransformsToWorld_.at(i-1)(0,0);
            rotationMatrixItoB(0,1) = jointTransformsToWorld_.at(i-1)(0,1);
            rotationMatrixItoB(0,2) = jointTransformsToWorld_.at(i-1)(0,2);
            
            //Row 2
            rotationMatrixItoB(1,0) = jointTransformsToWorld_.at(i-1)(1,0);
            rotationMatrixItoB(1,1) = jointTransformsToWorld_.at(i-1)(1,1);
            rotationMatrixItoB(1,2) = jointTransformsToWorld_.at(i-1)(1,2);
            //Row 3
            rotationMatrixItoB(2,0) = jointTransformsToWorld_.at(i-1)(2,0);
            rotationMatrixItoB(2,1) = jointTransformsToWorld_.at(i-1)(2,1);
            rotationMatrixItoB(2,2) = jointTransformsToWorld_.at(i-1)(2,2);
        }
        
        //Create the coiumn in the jacobian matrix
        jacobianLinearVelocityComponent = (rotationMatrixItoB * unitVector).cross((translationMatrixNtoB - translationMatrixItoB));
        jacobianRotationalVelocityComponent = rotationMatrixItoB * unitVector;

        //Row 1
        jacobianInWorldFrame_(0,i) = jacobianLinearVelocityComponent(0,0);
        //Row 2
        jacobianInWorldFrame_(1,i) = jacobianLinearVelocityComponent(1,0);
        //Row 3
        jacobianInWorldFrame_(2,i) = jacobianLinearVelocityComponent(2,0);
        //Row 4
        jacobianInWorldFrame_(3,i) = jacobianRotationalVelocityComponent(0,0);
        //Row 5
        jacobianInWorldFrame_(4,i) = jacobianRotationalVelocityComponent(1,0);
        //Row 6
        jacobianInWorldFrame_(5,i) = jacobianRotationalVelocityComponent(2,0);
    }
    ROS_DEBUG_STREAM("JACOBIAN IN WORLD FRAME");
    ROS_DEBUG_STREAM("\n" << jacobianInWorldFrame_);
}

void Robot::calculateJacobianInEndEffectorFrame(){
    
    Eigen::MatrixXd alterationMatrix(6,6);
    Eigen::MatrixXd inverseEndEffectorTransformation = getJointTransformToWorld(5).inverse();

    //Row 1
    alterationMatrix(0,0) = inverseEndEffectorTransformation(0,0);
    alterationMatrix(0,1) = inverseEndEffectorTransformation(0,1);
    alterationMatrix(0,2) = inverseEndEffectorTransformation(0,2);
    alterationMatrix(0,3) = 0;
    alterationMatrix(0,4) = 0;
    alterationMatrix(0,5) = 0;
    //Row 2
    alterationMatrix(1,0) = inverseEndEffectorTransformation(1,0);
    alterationMatrix(1,1) = inverseEndEffectorTransformation(1,1);
    alterationMatrix(1,2) = inverseEndEffectorTransformation(1,2);
    alterationMatrix(1,3) = 0;
    alterationMatrix(1,4) = 0;
    alterationMatrix(1,5) = 0;
    //Row 3
    alterationMatrix(2,0) = inverseEndEffectorTransformation(2,0);
    alterationMatrix(2,1) = inverseEndEffectorTransformation(2,1);
    alterationMatrix(2,2) = inverseEndEffectorTransformation(2,2);
    alterationMatrix(2,3) = 0;
    alterationMatrix(2,4) = 0;
    alterationMatrix(2,5) = 0;
    //Row 4
    alterationMatrix(3,0) = 0;
    alterationMatrix(3,1) = 0;
    alterationMatrix(3,2) = 0;
    alterationMatrix(3,3) = inverseEndEffectorTransformation(0,0);
    alterationMatrix(3,4) = inverseEndEffectorTransformation(0,1);
    alterationMatrix(3,5) = inverseEndEffectorTransformation(0,2);
    //Row 5
    alterationMatrix(4,0) = 0;
    alterationMatrix(4,1) = 0;
    alterationMatrix(4,2) = 0;
    alterationMatrix(4,3) = inverseEndEffectorTransformation(1,0);
    alterationMatrix(4,4) = inverseEndEffectorTransformation(1,1);
    alterationMatrix(4,5) = inverseEndEffectorTransformation(1,2);
    //Row 6
    alterationMatrix(5,0) = 0;
    alterationMatrix(5,1) = 0;
    alterationMatrix(5,2) = 0;
    alterationMatrix(5,3) = inverseEndEffectorTransformation(2,0);
    alterationMatrix(5,4) = inverseEndEffectorTransformation(2,1);
    alterationMatrix(5,5) = inverseEndEffectorTransformation(2,2);
    
    jacobianInEndEffectorFrame_ = alterationMatrix * jacobianInWorldFrame_;

    ROS_DEBUG_STREAM("JACOBIAN IN END EFFECTOR FRAME");
    ROS_DEBUG_STREAM("\n" << jacobianInEndEffectorFrame_);
}

void Robot::calculatePseudoInverseJacobianInEndEffectorFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);

    Eigen::MatrixXd transposeJacobian;
    double damping;
    
    if(measureOfManipubilityInEndEffectorFrame_ > dampingThreshold_){
        damping = 0;
    }
    else{
        damping = (1 - pow((measureOfManipubilityInEndEffectorFrame_/dampingThreshold_),2))* dampingMax_;
    }
        
    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(jacobianInEndEffectorFrame_.rows(), jacobianInEndEffectorFrame_.cols()) ;

    transposeJacobian = jacobianInEndEffectorFrame_.transpose();
    pseudoInverseJacobianInEndEffectorFrame_ = transposeJacobian * (jacobianInEndEffectorFrame_*transposeJacobian + damping * identityMatrix).inverse();
    ROS_DEBUG_STREAM("PSUEDO INVERSE JACOBIAN IN END EFFECTOR FRAME \n" << pseudoInverseJacobianInEndEffectorFrame_);
}

void Robot::calculatePseudoInverseJacobianInWorldFrame(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);

    Eigen::MatrixXd transposeJacobian;
    double damping;
    
    if(measureOfManipubilityInWorldFrame_ > dampingThreshold_){
        damping = 0;
    }
    else{
        damping = (1 - pow((measureOfManipubilityInWorldFrame_/dampingThreshold_),2))* dampingMax_;
    }
        
    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(jacobianInWorldFrame_.rows(), jacobianInWorldFrame_.cols()) ;

    transposeJacobian = jacobianInEndEffectorFrame_.transpose();
    pseudoInverseJacobianInWorldFrame_ = transposeJacobian * (jacobianInWorldFrame_*transposeJacobian + damping * identityMatrix).inverse();
    ROS_DEBUG_STREAM("PSUEDO INVERSE JACOBIAN IN END EFFECTOR FRAME \n" << pseudoInverseJacobianInWorldFrame_);
}


void Robot::calculateMeasureOfManipulabilityInEndEffector(){
    measureOfManipubilityInEndEffectorFrame_ = sqrt(((jacobianInEndEffectorFrame_ * jacobianInEndEffectorFrame_.transpose()).determinant()));
    ROS_DEBUG_STREAM("MEASURE OF MANIPULABILITY IN END EFFECTOR FRAME \n" << measureOfManipubilityInEndEffectorFrame_);
}

