#include <gtest/gtest.h>
#include "../src/robotController.h"

TEST(RobotController,testCompile){
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta);

    double gain = 0.1;
    double errorThreshold = 1;

    RobotController robotController(nh,&robot,gain,errorThreshold);

    Eigen::Vector3d fiducialTranslationLocal;

    //X
    fiducialTranslationLocal(0,0) = 0.5;

    //Y
    fiducialTranslationLocal(1,0) = 0.5;

    //Z
    fiducialTranslationLocal(2,0) = 0.5;


    float roll = M_PI/2, pitch = 0, yaw = M_PI/4;    
    Eigen::Quaterniond fiducialRotationLocal;
    fiducialRotationLocal = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
   robotController.setFiducialPostition(fiducialTranslationLocal,fiducialRotationLocal);
   
}

TEST(RobotController,tesEndEffectorVelocity){
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta);

    robot.calculateJointTransforms();
    robot.calculateJointTransformsToBase();
    robot.calculateJacobian(); 

    double gain = 0.1;
    double errorThreshold = 1;

    RobotController robotController(nh,&robot,gain,errorThreshold);

    Eigen::Vector3d fiducialTranslationLocal;

    //X
    fiducialTranslationLocal(0,0) = 0.5;

    //Y
    fiducialTranslationLocal(1,0) = 0.5;

    //Z
    fiducialTranslationLocal(2,0) = 0.5;


    float roll = M_PI/2, pitch = 0, yaw = 0;    
    Eigen::Quaterniond fiducialRotationLocal;
    fiducialRotationLocal.w() = 0.1;
    fiducialRotationLocal.x() = 0.2;
    fiducialRotationLocal.y() = 0.3;
    fiducialRotationLocal.z() = 0.4;
    
    robotController.setFiducialPostition(fiducialTranslationLocal,fiducialRotationLocal);

    ROS_INFO_STREAM(fiducialRotationLocal.w() << " " << fiducialRotationLocal.x() << " " << fiducialRotationLocal.y() << " " << fiducialRotationLocal.z() << " ");
    robotController.calculateEndEffectorVelocity();

    ASSERT_NEAR(robotController.getEndEffectorVelocity()(0,0),0.01,0.001);
    ASSERT_NEAR(robotController.getEndEffectorVelocity()(1,0),0.02,0.001);
    ASSERT_NEAR(robotController.getEndEffectorVelocity()(2,0),0.03,0.001);
    ASSERT_NEAR(robotController.getEndEffectorVelocity()(3,0),-0.1373,0.001);
    ASSERT_NEAR(robotController.getEndEffectorVelocity()(4,0),0.0823,0.001);
    ASSERT_NEAR(robotController.getEndEffectorVelocity()(5,0),-0.2944,0.001);
}




int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "visual_servoing_robot_tests");
    return RUN_ALL_TESTS();
}