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
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta);
   
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
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta);
   
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
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta); 
    
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();
    
    //Calculate the jacobian
    robot.calculateJacobian();

    ROS_INFO_STREAM("/n" << robot.getJacobian());

    //Row 1
    ASSERT_NEAR(robot.getJacobian()(0,0),0.1943,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,1),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,2),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,3),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,4),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,5),0,0.01);
    //Row 2()
    ASSERT_NEAR(robot.getJacobian()(1,0),-0.4569 ,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,5),0,0.01);
    //Row 3()
    ASSERT_NEAR(robot.getJacobian()(2,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,1),-0.4569 ,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,2),-0.2132,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,5),0,0.01);
    //Row 4()
    ASSERT_NEAR(robot.getJacobian()(3,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,5),0,0.01);
    //Row 5()
    ASSERT_NEAR(robot.getJacobian()(4,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,1),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,2),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,5),-1,0.01);
    //Row 6
    ASSERT_NEAR(robot.getJacobian()(5,0),1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,4),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,5),0,0.01);
}

TEST(Robot,testJacobianTranspose){
    //Create a robot
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta); 
    
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();
    
    //Calculate the jacobian
    robot.calculateJacobian();

    ROS_INFO_STREAM("/n" << robot.getJacobian());

    //Row 1
    ASSERT_NEAR(robot.getJacobian()(0,0),0.1943,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,1),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,2),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,3),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,4),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,5),0,0.01);
    //Row 2()
    ASSERT_NEAR(robot.getJacobian()(1,0),-0.4569 ,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,5),0,0.01);
    //Row 3()
    ASSERT_NEAR(robot.getJacobian()(2,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,1),-0.4569 ,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,2),-0.2132,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,5),0,0.01);
    //Row 4()
    ASSERT_NEAR(robot.getJacobian()(3,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,5),0,0.01);
    //Row 5()
    ASSERT_NEAR(robot.getJacobian()(4,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,1),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,2),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,5),-1,0.01);
    //Row 6
    ASSERT_NEAR(robot.getJacobian()(5,0),1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,4),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,5),0,0.01);
}

TEST(Robot,testJacobianPseudoInverse){
    //Create a robot
    ros::NodeHandle nh;
    
    std::vector<double> d{0.1519,0,0,0.11235,0.08535,0.0819};
    std::vector<double> a{0,-0.24365,-0.21325,0,0,0};
    std::vector<double> alpha{M_PI/2,0,0,M_PI/2,-M_PI/2,0};
    Robot robot(nh,d,a,alpha);
    
    //Lets set the joint angles and compare with values obtained from Matlab
    std::vector<double> theta{0,0,0,0,0,0};
    robot.setTheta(theta); 
    
    //Now lets calculate the joint transforms
    robot.calculateJointTransforms();
    
    //Calculate the joint transforms with respect to the base
    robot.calculateJointTransformsToBase();
    
    //Calculate the jacobian
    robot.calculateJacobian();

    ROS_INFO_STREAM("/n" << robot.getJacobian());

    //Row 1
    ASSERT_NEAR(robot.getJacobian()(0,0),0.1943,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,1),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,2),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,3),0.0853,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,4),-0.0819,0.01);
    ASSERT_NEAR(robot.getJacobian()(0,5),0,0.01);
    //Row 2()
    ASSERT_NEAR(robot.getJacobian()(1,0),-0.4569 ,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(1,5),0,0.01);
    //Row 3()
    ASSERT_NEAR(robot.getJacobian()(2,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,1),-0.4569 ,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,2),-0.2132,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(2,5),0,0.01);
    //Row 4()
    ASSERT_NEAR(robot.getJacobian()(3,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(3,5),0,0.01);
    //Row 5()
    ASSERT_NEAR(robot.getJacobian()(4,0),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,1),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,2),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,3),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,4),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(4,5),-1,0.01);
    //Row 6
    ASSERT_NEAR(robot.getJacobian()(5,0),1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,1),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,2),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,3),0,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,4),-1,0.01);
    ASSERT_NEAR(robot.getJacobian()(5,5),0,0.01);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "visual_servoing_robot_tests");
    return RUN_ALL_TESTS();
}