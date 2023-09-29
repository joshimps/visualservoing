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

    robot_model_loader::RobotModelLoader robot_model_loader("ur3_robot");
    kinematic_model_ = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());
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

tf2::Transform Robot::calculateJointTransforms(){
    std::unique_lock<std::mutex> lck(jointStateMutex_);
    tf2::Transform tRz;
    tf2::Transform tz;
    tf2::Transform tx;
    tf2::Transform tRx;

    for(int i = 0; i < jointStates_.position.size(); i++){

        tf2::Vector3 tRztranslation(0,0,0);
        tf2::Matrix3x3 tRzrotation(tf2Cos(jointStates_.position.at(i)),-tf2Sin(jointStates_.position.at(i)),0,
                                   tf2Sin(jointStates_.position.at(i)),tf2Cos(jointStates_.position.at(i)),0,
                                   0,0,1 );
        tRz.setOrigin(tRztranslation);
        tRz.setBasis(tRzrotation);

        tf2::Vector3 tztranslation(0,0,d_.at(i));
        tf2::Matrix3x3 tzrotation(1,0,0,
                                  0,1,0,
                                  0,0,1);
        tz.setOrigin(tztranslation);
        tz.setBasis(tzrotation);


        tf2::Vector3 txtranslation(a_.at(i),0,0);
        tf2::Matrix3x3 txrotation(1,0,0,
                                  0,1,0,
                                  0,0,1);
        tx.setOrigin(txtranslation);
        tx.setBasis(txrotation);

        tf2::Vector3 tRxtranslation(0,0,0);
        tf2::Matrix3x3 tRxrotation(1,0,0,
                                   0,tf2Cos(jointStates_.position.at(i)),-tf2Sin(jointStates_.position.at(i)),
                                   0,tf2Sin(jointStates_.position.at(i)),tf2Cos(jointStates_.position.at(i)));
        tRx.setOrigin(tRxtranslation);
        tRx.setBasis(tRxrotation);

        jointTransforms_.at(i) = tRz * tz * tx * tRx;
    }   
}
