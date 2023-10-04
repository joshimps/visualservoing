#include <gtest/gtest.h>
#include "../src/robotController.h"

TEST(RobotController,testEndEffectorVelocity){
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
   robotController.moveRobot();
    
}

TEST(RobotController,testJointVelocity){


}

TEST(RobotController,testStable){

}

TEST(RobotController,testConverging){

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "visual_servoing_robot_tests");
    return RUN_ALL_TESTS();
}