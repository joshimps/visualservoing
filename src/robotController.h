#include "robot.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf2/LinearMath/Transform.h"
#include <vector>
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "Eigen/Dense"

class RobotController{
    public:

    protected:

    private:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////

    RobotController(ros::NodeHandle nh, Robot* robot, double gain, double errorThreshold);
    ~RobotController();

    ///////////////////////////////////////////////////////////
    // Calculation
    //////////////////////////////////////////////////////////

    void RobotController::moveRobot();
    
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