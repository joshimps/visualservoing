#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <mutex>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <thread>

class Robot
{
public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief Robot Constructor
     *
     *   @param nh ros node handle
     *   @param d values for dh parameter d in order from base to end effector
     *   @param a values for dh parameter a in order from base to end effector
     *   @param alpha values for dh parameter alpha in order from base to end effector
     *   @param jointNames name of robot joints in order from base to end effector
     *
     *   @return void
     */
    Robot(ros::NodeHandle nh, std::vector<double> d, std::vector<double> a, std::vector<double> alpha, std::vector<std::string> jointNames);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Getters
    /////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief Gets the current robots joint state
     *
     *   @return sensor_msgs::JointState the currently held message reported by /joint_states
     */
    sensor_msgs::JointState getJointState();

    /*! @brief Gets the robots current end effector transform with respect to the base
     *
     *   @return Eigen::MatrixXd urrent end effector transform with respect to the base
     */
    Eigen::MatrixXd getEndEffectorTransform();

    /*! @brief Gets the robots joint transform at index i
     *
     *   @param i Index of the joint transform desired
     *
     *   @return Eigen::MatrixXd joint transform at index i
     */
    Eigen::MatrixXd getJointTransform(int i);

    /*! @brief Gets the robots joint transform at index i in respect to the robots base transform
     *
     *   @param i Index of the joint transform desired
     *
     *   @return Eigen::MatrixXd joint transform at index i in respect to the robots base transform
     */
    Eigen::MatrixXd getJointTransformToBase(int i);

    /*! @brief Gets the robots joint transform at index i in respect to the world transform
     *
     *   @param i Index of the joint transform desired
     *
     *   @return Eigen::MatrixXd joint transform at index i in respect to the world transform
     */
    Eigen::MatrixXd getJointTransformToWorld(int i);

    /*! @brief Gets the robots base transform
     *
     *   @return Eigen::MatrixXd robots base transform
     */
    Eigen::MatrixXd getBaseTransform();

    /*! @brief Gets the current jacobian held by the propert jacobianInWorldFrame_
     *
     *   @return Eigen::MatrixXd jacobian in world frame
     */
    Eigen::MatrixXd getJacobianInWorldFrame();

    /*! @brief Gets the current transpose jacobian based on the current jacobian held by the property jacobianInWorldFrame_
     *
     *   @return Eigen::MatrixXd transpose jacobian in world frame
     */
    Eigen::MatrixXd getTransposeJacobianInWorldFrame();

    /*! @brief Gets the current psuedo inverse jacobian held by the property pseudoInverseJacobianInWorldFrame_
     *
     *   @return Eigen::MatrixXd psuedo inverse jacobian in world frame
     */
    Eigen::MatrixXd getPseudoInverseJacobianInWorldFrame();

    /*! @brief Gets the current jacobian held by the propert jacobianInEndEffectorFrame_
     *
     *   @return Eigen::MatrixXd jacobian in end effector frame
     */
    Eigen::MatrixXd getJacobianInEndEffectorFrame();

    /*! @brief Gets the current transpose jacobian based on the current jacobian held by the property jacobianInEndEffectorFrame_
     *
     *   @return Eigen::MatrixXd transpose jacobian in end effector frame
     */
    Eigen::MatrixXd getTransposeJacobianInEndEffectorFrame();

    /*! @brief Gets the current psuedo inverse jacobian held by the property pseudoInverseJacobianInEndEffectorFrame_
     *
     *   @return Eigen::MatrixXd psuedo inverse jacobian in end effector frame
     */
    Eigen::MatrixXd getPseudoInverseJacobianInEndEffectorFrame();

    /*! @brief Gets the number of joints of this robot
     *
     *   @return int number of joints in this robot
     */
    int getNumberOfJoints();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Setters
    /////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief Sets the theta value in dh parameters, useful for testing, do not use otherwise
     *
     *   @param theta values for dh parameter theta in order from base to end effector
     *   @param jointNames name of robot joints in order from base to end effector
     *   @return void
     */
    void setTheta(std::vector<double> theta, std::vector<std::string> jointNames);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Calculations
    /////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief Calculates the current joint transforms for this robot and saves them in jointTransforms_
     *
     *   @return void
     */
    void calculateJointTransforms();

    /*! @brief Calculates the current joint transforms for this robot in reference to the base frame and saves them in jointTransformsToBase_
     *
     *   @return void
     */
    void calculateJointTransformsToBase();

    /*! @brief Calculates the current joint transforms for this robot in reference to the world frame and saves them in jointTransformsToWorld_
     *
     *   @return void
     */
    void calculateJointTransformsToWorld();

    /*! @brief Calculates the current jacobian in the world frame for this robot and saves them in jacobianInWorldFrame_
     *
     *   @return void
     */
    void calculateJacobianInWorldFrame();

    /*! @brief Calculates the current jacobian in the end effector frame for this robot and saves them in jacobianInEndEffectorFrame_
     *
     *   @return void
     */
    void calculateJacobianInEndEffectorFrame();

    /*! @brief Calculates the current measure of manipulability in the end effector frame for this robot and saves them in measureOfManipubilityInEndEffectorFrame_
     *
     *   @return void
     */
    void calculateMeasureOfManipulabilityInEndEffector();

    /*! @brief Calculates the current pseudo inverse jacobian in the world frame for this robot and saves them in pseudoInverseJacobianInWorldFrame_
     *
     *   @return void
     */
    void calculatePseudoInverseJacobianInWorldFrame();

    /*! @brief Calculates the current pseudo inverse jacobian in the end effector frame for this robot and saves them in pseudoInverseJacobianInEndEffectorFrame_
     *
     *   @return void
     */
    void calculatePseudoInverseJacobianInEndEffectorFrame();

private:
    ///////////////////////////////////////////////////////////
    // Callbacks
    //////////////////////////////////////////////////////////

    /*! @brief Callback that saves a message published on /joint_states to jointStates_
     *
     *   @param msg message from /joint_states
     *
     *   @return void
     */
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
    std::vector<Eigen::Matrix4d> jointTransformsToWorld_;
    Eigen::MatrixXd jacobianInWorldFrame_;
    Eigen::MatrixXd jacobianInEndEffectorFrame_;
    Eigen::MatrixXd pseudoInverseJacobianInWorldFrame_;
    Eigen::MatrixXd pseudoInverseJacobianInEndEffectorFrame_;
    double measureOfManipubilityInWorldFrame_;
    double measureOfManipubilityInEndEffectorFrame_;
    double dampingThreshold_;
    double dampingMax_;
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