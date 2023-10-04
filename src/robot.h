#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include <atomic>
#include <mutex>
#include <vector>
#include "eigen3/Eigen/Dense"

class Robot{
    public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////
    Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha);
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    /////////////////////////////////////////////////////////////////////////////////////////
    sensor_msgs::JointState getJointState();
    Eigen::MatrixXd getEndEffectorTransform();
    Eigen::MatrixXd getJointTransform(int i);
    Eigen::MatrixXd getJointTransformToBase(int i);
    Eigen::MatrixXd getJacobian();
    int getNumberOfJoints();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Setters
    /////////////////////////////////////////////////////////////////////////////////////////
    void setTheta(std::vector<double> theta);


    ///////////////////////////////////////////////////////////////////////////////////////////
    // Calculations
    /////////////////////////////////////////////////////////////////////////////////////////

    void calculateJointTransforms();
    void calculateJacobian();
    void calculateJointTransformsToBase();
    
    protected:

    private:
    
    ///////////////////////////////////////////////////////////
    // Callbacks
    //////////////////////////////////////////////////////////
    void jointStateCallBack(const sensor_msgs::JointStateConstPtr &msg);

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
};