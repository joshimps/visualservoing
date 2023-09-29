#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include <atomic>
#include <mutex>
#include <vector>

class Robot{
    public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    /////////////////////////////////////////////////////////////////////////////////////////
    sensor_msgs::JointState getJointState();
    tf2::Transform getEndEffectorTransform();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Calculations
    /////////////////////////////////////////////////////////////////////////////////////////

    tf2::Transform calculateJointTransforms();

    protected:

    private:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////
    Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha);
    ~Robot();
    
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
    // Joints
    /////////////////////////////////////////////////////////////////////
    
    sensor_msgs::JointState jointStates_;
    std::vector<tf2::Transform> jointTransforms_;

    ///////////////////////////////////////////////////////////////////////
    // Robot DH Params
    /////////////////////////////////////////////////////////////////////

    std::vector<double> theta_;
    std::vector<double> d_;
    std::vector<double> a_;
    std::vector<double> alpha_;
};