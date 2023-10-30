#include "robot.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "rosgraph_msgs/Clock.h"
#include <atomic>
#include <mutex>
#include <vector>
#include <thread>

class RobotController{
    public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief Constructor of the RobotController class
     *
     *   @param nh ros node handle
     *   @param robot robot to control
     *   @param gain gain of the p controller
     *   @param errorThreshold threshold which when under the controller should consider itself done
     *   @return void
     */
    RobotController(ros::NodeHandle nh, Robot* robot, double gain, double errorThreshold);

    ///////////////////////////////////////////////////////////
    // Action
    //////////////////////////////////////////////////////////

    /*! @brief Calculates the joint velocities required to move to the fiducials location and publishs them to the robot controller
     *
     *   @return void
     */
    void moveRobot();

     /*! @brief Stalls the robot, writing 0 as the joint velocity and publishes them to the robot controller
     *
     *   @return void
     */
    void stallRobot();

    /*! @brief Calculates the end effector velocity required to reach the fiducials location
     *
     *   @return void
     */
    void calculateEndEffectorVelocity();


    ///////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    /////////////////////////////////////////////////////////////////////////////////////////

     /*! @brief Gets the end effector velocity
     *
     *   @return Eigen::VectorXd end effector velocity in translational xyz and rotational quat xyz components
     */
    Eigen::VectorXd getEndEffectorVelocity();
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Setters
    /////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief Sets a position for the fiducial to be, useful for testing, do not use otherwise
     *
     *   @param fiducialTranslationLocal matrix containing the xyz translational component of where the fiducial should be
     *   @param fiducialRotationLocal matrix containing the quat xyz rotational component of where the fiducial should be
     * 
     *   @return void
     */ 
    void setFiducialPostition(Eigen::MatrixXd fiducialTranslationLocal , Eigen::Quaterniond fiducialRotationLocal);

    private:
   
    ///////////////////////////////////////////////////////////
    // Callbacks and Services
    //////////////////////////////////////////////////////////

    void fiducialPositionCallBack(const geometry_msgs::PoseStampedPtr &msg);
    void clockCallback(const rosgraph_msgs::ClockConstPtr &msg);

    ///////////////////////////////////////////////////////////////////////
    // Node, Publishers and Subscribers
    /////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh_;
    ros::Subscriber fiducialPositionSub_;
    ros::Subscriber clockSub_;
    ros::Publisher jointVelocityPub_;
    ros::Publisher endEffectorVelocityPub_;
    ros::Publisher euclidianNormPub_;

    ///////////////////////////////////////////////////////////////////////
    // Data Security and Pointers
    /////////////////////////////////////////////////////////////////////
    std::mutex fiducialPoseMutex_;

    ///////////////////////////////////////////////////////////////////////
    // Controller Parameters
    /////////////////////////////////////////////////////////////////////
    double gain_;
    double errorThreshold_;
    double euclidianNorm_;

    ///////////////////////////////////////////////////////////////////////
    // Variables
    /////////////////////////////////////////////////////////////////////
    
    Eigen::Vector3d fiducialTranslationLocal_;
    Eigen::Quaterniond fiducialRotationLocal_;
    Eigen::VectorXd endEffectorVelocity_;
    ros::Time timeAtFiducialPublish_;

    ///////////////////////////////////////////////////////////////////////
    // Robot To Control Velocity
    /////////////////////////////////////////////////////////////////////
    Robot* robot_;

    ///////////////////////////////////////////////////////////////////////
    // Flags
    /////////////////////////////////////////////////////////////////////
    bool recievedFiducial_;
};