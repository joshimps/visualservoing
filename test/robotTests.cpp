#include <gtest/gtest.h>
#include <vector>
#include "ros/ros.h"
#include "../src/robot.h"
#include <cmath>

TEST(Robot,testJointTransforms){

    //Create a robot
    ros::NodeHandle nh;
    //Testing from ubuntu
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    std::vector<std::string> jointNames = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    Robot robot(nh,d,a,alpha,jointNames);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta,jointNames); 

   
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Compare each transform with accepted value from Matlab

    //Link 1

    //Row 1
    ASSERT_NEAR(robot.getJointTransform(0)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(0,3),0,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransform(0)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransform(0)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(2,3),0.1519,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransform(0)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(0)(3,3),1,0.01);
    
    //Link 2
    
    //Row 1
    ASSERT_NEAR(robot.getJointTransform(1)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(0,3),-0.2437,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransform(1)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(1,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(1,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransform(1)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(2,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(2,2),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(2,3),0,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransform(1)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(1)(3,3),1,0.01);

    //Link 3

    //Row 1
    ASSERT_NEAR(robot.getJointTransform(2)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(0,3),-0.2132,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransform(2)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(1,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(1,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransform(2)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(2,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(2,2),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(2,3),0,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransform(2)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(2)(3,3),1,0.01);

    //Link 4

    //Row 1
    ASSERT_NEAR(robot.getJointTransform(3)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(0,3),0,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransform(3)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransform(3)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(2,3),0.1124,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransform(3)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(3)(3,2),0,0.01); 
    ASSERT_NEAR(robot.getJointTransform(3)(3,3),1,0.01);

    //Link 5

   //Row 1
    ASSERT_NEAR(robot.getJointTransform(4)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(0,3),0,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransform(4)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(1,2),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransform(4)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(2,1),-1,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(2,3),0.0853,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransform(4)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(4)(3,3),1,0.01);

    //Link 6

    //Row 1
    ASSERT_NEAR(robot.getJointTransform(5)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(0,3),0,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransform(5)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(1,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(1,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransform(5)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(2,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(2,2),1,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(2,3),0.0819,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransform(5)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransform(5)(3,3),1,0.01);
}

TEST(Robot,testJointTransformsToBase){
    //Create a robot
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    std::vector<std::string> jointNames = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    Robot robot(nh,d,a,alpha,jointNames);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta,jointNames); 

   
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();

    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();


    //Check the values compared to known values from matlab

    //Link 1 to Base

    //Row 1
    ASSERT_NEAR(robot.getJointTransformToBase(0)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(0,3),0,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransformToBase(0)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransformToBase(0)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(2,3),0.1519,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,3),1,0.01);

    //Link 2 to Base

    //Row 1
    ASSERT_NEAR(robot.getJointTransformToBase(1)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(0,3),-0.2437,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransformToBase(1)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransformToBase(1)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(2,3),0.1519,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransformToBase(1)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(1)(3,3),1,0.01);

    //Link 3 to Base

    //Row 1
    ASSERT_NEAR(robot.getJointTransformToBase(2)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(0,3),-0.4569,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransformToBase(2)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(1,3),0,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransformToBase(2)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(2,3),0.1519,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransformToBase(2)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(2)(3,3),1,0.01);

    //Link 4 to Base

    //Row 1
    ASSERT_NEAR(robot.getJointTransformToBase(3)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(0,3),-0.4569,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransformToBase(3)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(1,1),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(1,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(1,3),-0.1124,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransformToBase(3)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(2,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(2,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(2,3),0.1519,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransformToBase(3)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(3)(3,3),1,0.01);

    //Link 5 to Base

    //Row 1
    ASSERT_NEAR(robot.getJointTransformToBase(4)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(0,3),-0.4569,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransformToBase(4)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(1,3),-0.1124,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransformToBase(4)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(2,3),0.0666,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransformToBase(4)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(4)(3,3),1,0.01);

    //Link 6 to Base

    //Row 1
    ASSERT_NEAR(robot.getJointTransformToBase(5)(0,0),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(0,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(0,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(0,3),-0.4569,0.01);
    //Row 2
    ASSERT_NEAR(robot.getJointTransformToBase(5)(1,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(1,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(1,2),-1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(1,3),-0.1943,0.01);
    //Row 3
    ASSERT_NEAR(robot.getJointTransformToBase(5)(2,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(2,1),1,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(2,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(5)(2,3),0.0666,0.01);
    //Row 4
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,0),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,1),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,2),0,0.01);
    ASSERT_NEAR(robot.getJointTransformToBase(0)(3,3),1,0.01);
}

TEST(Robot,testJacobian){
    //Create a robot
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    std::vector<std::string> jointNames = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    Robot robot(nh,d,a,alpha,jointNames);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{3*M_PI/2, 3*M_PI/2, M_PI/2, M_PI, 3*M_PI/2,0};
    robot.setTheta(theta,jointNames); 

    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();

    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToWorld();
    
    //Calculate the jacobian
    robot.calculateJacobianInWorldFrame();

    robot.calculateJacobianInEndEffectorFrame();

    ASSERT_NEAR(robot.getJacobianInWorldFrame()(0,0),0.29515,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(0,1),-4.2364e-17,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(0,2),2.3942e-18,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(0,3),-1.0664e-17,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(0,4),0.0819,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(0,5),0,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(1,0),0.11235,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(1,1),-0.329,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(1,2),-0.08535,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(1,3),-0.08535,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(1,4),-3.009e-17,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(1,5),0,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(2,0),-6.163e-33,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(2,1),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(2,2),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(2,3),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(2,4),4.9978e-33,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(2,5),0,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(3,0),1.2933e-32,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(3,1),1,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(3,2),1,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(3,3),1,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(3,4),7.3956e-32,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(3,5),-3.6739e-16,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(4,0),0,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(4,1),-1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(4,2),-1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(4,3),-1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(4,4),3.6739e-16,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(4,5),-1,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(5,0),1,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(5,1),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(5,2),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(5,3),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(5,4),1,0.01);
    ASSERT_NEAR(robot.getJacobianInWorldFrame()(5,5),4.2863e-16,0.01);

   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(0,0),-0.29515,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(0,1),-7.8509e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(0,2),-3.3751e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(0,3),-2.0693e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(0,4),-0.0819,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(0,5),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(1,0),-4.8156e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(1,1),0.29515,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(1,2),0.29515,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(1,3),0.0819,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(1,4),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(1,5),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(2,0),-0.11235,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(2,1),0.329,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(2,2),0.08535,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(2,3),0.08535,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(2,4),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(2,5),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(3,0),-7.3956e-32,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(3,1),-1,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(3,2),-1,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(3,3),-1,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(3,4),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(3,5),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(4,0),-1,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(4,1),-6.1232e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(4,2),-6.1232e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(4,3),-6.1232e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(4,4),-1,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(4,5),0,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(5,0),4.2863e-16,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(5,1),-1.837e-16,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(5,2),-1.837e-16,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(5,3),-1.837e-16,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(5,4),6.1232e-17,0.01);
   ASSERT_NEAR(robot.getJacobianInEndEffectorFrame()(5,5),1,0.01);    
}

TEST(Robot,testJacobianTranspose){
    //Create a robot
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    std::vector<std::string> jointNames = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    Robot robot(nh,d,a,alpha,jointNames);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{3*M_PI/2, 3*M_PI/2, M_PI/2, M_PI, 3*M_PI/2,0};
    robot.setTheta(theta,jointNames); 

    
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();

    robot.calculateJointTransformsToWorld();
    
    //Calculate the jacobian
    robot.calculateJacobianInWorldFrame();

    robot.calculateJacobianInEndEffectorFrame();


    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(0,0),-0.29515,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(0,1),-0.11235,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(0,2),-6.163e-33,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(0,3),-1.2933e-32,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(0,4),0,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(0,5),1,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(1,0),4.2364e-17,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(1,1),0.329,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(1,2),-0.29515,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(1,3),-1,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(1,4),1.837e-16,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(1,5),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(2,0),-2.3942e-18,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(2,1),0.08535,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(2,2),-0.29515,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(2,3),-1,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(2,4),1.837e-16,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(2,5),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(3,0),1.0664e-17,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(3,1),0.08535,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(3,2),-0.0819,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(3,3),-1,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(3,4),1.837e-16,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(3,5),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(4,0),-0.0819,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(4,1),3.009e-17,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(4,2),4.9978e-33,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(4,3),-7.3956e-32,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(4,4),-3.6739e-16,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(4,5),1,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(5,0),0,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(5,1),0,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(5,2),0,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(5,3),3.6739e-16,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(5,4),1,0.01);
    ASSERT_NEAR(robot.getTransposeJacobianInWorldFrame()(5,5),4.2863e-16,0.01);
}

TEST(Robot,testJacobianPseudoInverse){
    //Create a robot
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    std::vector<std::string> jointNames = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    Robot robot(nh,d,a,alpha,jointNames);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{3*M_PI/2, 3*M_PI/2, M_PI/2, M_PI, 3*M_PI/2,0};
    robot.setTheta(theta,jointNames); 
 
    
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();

    robot.calculateJointTransformsToWorld();
    
    //Calculate the jacobian
    robot.calculateJacobianInWorldFrame();

    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(0,0),-4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(0,1),2.4425e-15,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(0,2),7.7716e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(0,3),2.0817e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(0,4),1.6462e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(0,5),-0.38406,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(1,0),-2.1623,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(1,1),4.1042,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(1,2),-5.085e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(1,3),0.3503,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(1,4),-5.279e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(1,5),-0.17709,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(2,0),2.1623,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(2,1),-4.1042,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(2,2),-4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(2,3),0.033759,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(2,4),-8.831e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(2,5),0.17709,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(3,0),4.4357e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(3,1),-9.7719e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(3,2),4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(3,3),-1.3841,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(3,4),5.0849e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(3,5),6.772e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(4,0),4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(4,1),4.4409e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(4,2),3.3307e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(4,3),1.1102e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(4,4),-5.9324e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(4,5),1.3841,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(5,0),1.7228e-15,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(5,1),7.0521e-31,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(5,2),5.4772e-31,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(5,3),1.837e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(5,4),1,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobianInWorldFrame()(5,5),5.0849e-16,0.01);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "visual_servoing_robot_tests");
    return RUN_ALL_TESTS();
}