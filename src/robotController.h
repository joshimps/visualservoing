#include "robot.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <atomic>
#include <mutex>
#include <vector>
#include <thread>

class RobotController{
    public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////

    RobotController(ros::NodeHandle nh, Robot* robot, double gain, double errorThreshold);

    ///////////////////////////////////////////////////////////
    // Action
    //////////////////////////////////////////////////////////

    void moveRobot();
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Setters
    /////////////////////////////////////////////////////////////////////////////////////////

    void setFiducialPostition(Eigen::MatrixXd fiducialTranslationLocal , Eigen::Quaterniond fiducialRotationLocal);

    protected:

    private:
   
    ///////////////////////////////////////////////////////////
    // Callbacks and Services
    //////////////////////////////////////////////////////////

    void fiducialPositionCallBack(const geometry_msgs::PoseStampedPtr &msg);

    ///////////////////////////////////////////////////////////////////////
    // Node, Publishers and Subscribers
    /////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh_;
    ros::Subscriber fiducialPositionSub_;
    ros::Publisher jointVelocityPub_;

    ///////////////////////////////////////////////////////////////////////
    // Data Security and Pointers
    /////////////////////////////////////////////////////////////////////
    std::mutex fiducialPoseMutex_;

    ///////////////////////////////////////////////////////////////////////
    // Controller Parameters
    /////////////////////////////////////////////////////////////////////
    double gain_;
    double errorThreshold_;

    ///////////////////////////////////////////////////////////////////////
    // Variables
    /////////////////////////////////////////////////////////////////////
    
    Eigen::Vector3d fiducialTranslationLocal_;
    Eigen::Quaterniond fiducialRotationLocal_;

    ///////////////////////////////////////////////////////////////////////
    // Robot To Control Velocity
    /////////////////////////////////////////////////////////////////////
    Robot* robot_;

};