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
    
    //Calculate the jacobian
    robot.calculateJacobian();

    ROS_INFO_STREAM("/n" << robot.getJacobian());

    ASSERT_NEAR(robot.getJacobian()(0,0),-0.29515,0.01  );
    ASSERT_NEAR(robot.getJacobian()(0,1),4.2364e-17,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,2),-2.3942e-18,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,3),1.0664e-17,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,4),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,5),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,0),-0.11235,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,1),0.329,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,2),0.08535,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,3),0.08535,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,4),3.009e-17,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,5),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,0),-6.163e-33,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,1),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,2),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,3),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,4),4.9978e-33,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,5),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,0),-1.2933e-32,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,1),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,2),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,4),-7.3956e-32,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,5),3.6739e-16,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,1),1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,2),1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,3),1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,4),-3.6739e-16,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,5),1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,0),1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,1),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,2),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,3),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,4),1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,5),4.2863e-16,0.01);

    
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
    
    //Calculate the jacobian
    robot.calculateJacobian();

    ROS_INFO_STREAM("/n" << robot.getJacobian());

    ASSERT_NEAR(robot.getJacobian().transpose()(0,0),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(0,1),-0.11235,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(0,2),-6.163e-33,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(0,3),-1.2933e-32,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(0,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(0,5),1,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(1,0),4.2364e-17,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(1,1),0.329,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(1,2),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(1,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(1,4),1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(1,5),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(2,0),-2.3942e-18,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(2,1),0.08535,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(2,2),-0.29515,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(2,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(2,4),1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(2,5),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(3,0),1.0664e-17,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(3,1),0.08535,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(3,2),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(3,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(3,4),1.837e-16,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(3,5),6.1232e-17,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(4,0),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(4,1),3.009e-17,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(4,2),4.9978e-33,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(4,3),-7.3956e-32,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(4,4),-3.6739e-16,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(4,5),1,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(5,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(5,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(5,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(5,3),3.6739e-16,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(5,4),1,0.01);
    ASSERT_NEAR(robot.getJacobian().transpose()(5,5),4.2863e-16,0.01);
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
    
    //Calculate the jacobian
    robot.calculateJacobian();

    
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(0,0),-4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(0,1),2.4425e-15,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(0,2),7.7716e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(0,3),2.0817e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(0,4),1.6462e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(0,5),-0.38406,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(1,0),-2.1623,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(1,1),4.1042,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(1,2),-5.085e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(1,3),0.3503,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(1,4),-5.279e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(1,5),-0.17709,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(2,0),2.1623,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(2,1),-4.1042,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(2,2),-4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(2,3),0.033759,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(2,4),-8.831e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(2,5),0.17709,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(3,0),4.4357e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(3,1),-9.7719e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(3,2),4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(3,3),-1.3841,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(3,4),5.0849e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(3,5),6.772e-17,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(4,0),4.6893,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(4,1),4.4409e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(4,2),3.3307e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(4,3),1.1102e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(4,4),-5.9324e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(4,5),1.3841,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(5,0),1.7228e-15,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(5,1),7.0521e-31,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(5,2),5.4772e-31,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(5,3),1.837e-16,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(5,4),1,0.01);
    ASSERT_NEAR(robot.getPseudoInverseJacobian()(5,5),5.0849e-16,0.01);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "visual_servoing_robot_tests");
    return RUN_ALL_TESTS();
}