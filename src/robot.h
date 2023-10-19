#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <mutex>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <thread>

class Robot{
    public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////
    Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha, std::vector<std::string> jointNames);
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    /////////////////////////////////////////////////////////////////////////////////////////
    sensor_msgs::JointState getJointState();
    Eigen::MatrixXd getEndEffectorTransform();
    Eigen::MatrixXd getJointTransform(int i);
    Eigen::MatrixXd getJointTransformToBase(int i);
    Eigen::MatrixXd getBaseTransform();
    Eigen::MatrixXd getJacobian();
    Eigen::MatrixXd getTransposeJacobian();
    Eigen::MatrixXd getPseudoInverseJacobian();
    int getNumberOfJoints();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Setters
    /////////////////////////////////////////////////////////////////////////////////////////
    void setTheta(std::vector<double> theta,std::vector<std::string> jointNames);


    ///////////////////////////////////////////////////////////////////////////////////////////
    // Calculations
    /////////////////////////////////////////////////////////////////////////////////////////

    void calculateJointTransforms();
    void calculateJacobian();
    void calculateJointTransformsToBase();

    ///////////////////////////////////////////////////////////
    // Callbacks
    //////////////////////////////////////////////////////////
    void jointStateCallBack(const sensor_msgs::JointStateConstPtr &msg);
    
    protected:

    private:
    
    ///////////////////////////////////////////////////////////////////////
    // Node, Publishers and Subscribers
    /////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh_;
    ros::Subscriber jointStateSub_;

    ///////////////////////////////////////////////////////////////////////
    // Data Security and Pointers
    /////////////////////////////////////////////////////////////////////
    std::mutex jointStateMutex_;

    ///////////////////////////////////////////////////////////////////////
    // Base and Joints
    /////////////////////////////////////////////////////////////////////
    Eigen::Matrix4d baseTransform_;
    sensor_msgs::JointState jointStates_;
    std::vector<Eigen::Matrix4d> jointTransforms_;
    std::vector<Eigen::Matrix4d> jointTransformsToBase_;
    Eigen::MatrixXd jacobian_;
    int numberOfJoints_;

    ///////////////////////////////////////////////////////////////////////
    // Robot DH Params
    /////////////////////////////////////////////////////////////////////

    std::vector<double> theta_;
    std::vector<double> d_;
    std::vector<double> a_;
    std::vector<double> alpha_;
    std::vector<std::string> jointNames_;
};