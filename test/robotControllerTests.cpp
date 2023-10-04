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
}

TEST(RobotController,testJointVelocity){
    

}

TEST(RobotController,testStable){

}

TEST(RobotController,testConverging){

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}