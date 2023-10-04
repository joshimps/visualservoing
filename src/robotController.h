#include "robot.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
#include <atomic>
#include <mutex>
#include <vector>

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

    void setFiducialPostition(geometry_msgs::PoseWithCovariancePtr &msg);

    protected:

    private:
   
    ///////////////////////////////////////////////////////////
    // Callbacks and Services
    //////////////////////////////////////////////////////////

    void fiducialPositionCallBack(geometry_msgs::PoseWithCovariancePtr &msg);

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
    
    Eigen::MatrixXd fiducialTranslationLocal_;
    Eigen::Quaterniond fiducialRotationLocal_;

    ///////////////////////////////////////////////////////////////////////
    // Robot To Control Velocity
    /////////////////////////////////////////////////////////////////////
    Robot* robot_;

};