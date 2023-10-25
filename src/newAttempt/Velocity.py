#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_multiply
import numpy as np
from math import pi

## CFG ##
gain = 0.05
refQuat = PoseStamped()
refQuat.pose.orientation.x = 0
refQuat.pose.orientation.y = 1
refQuat.pose.orientation.z = 0
refQuat.pose.orientation.w = 0

def get_transform_wrist():
    tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
    tf2_ros.TransformListener(tf_buffer)
    try:
        transformation = tf_buffer.lookup_transform('base_link', 'wrist_3_link', rospy.Time(0), rospy.Duration(0.2))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s' % 'base', 'wrist')



def procFiducial(msg):
    try:
        wrist_tf = get_transform_wrist()
    except Exception as e:
        raise(e)
        return 0

    velMsg = Float32MultiArray()
    xVel = msg.pose.position.x - wrist_tf.transform.translation.x 
    yVel = (msg.pose.position.y - wrist_tf.transform.translation.y)
    zVel = msg.pose.position.z - wrist_tf.transform.translation.z
    
    velMsg.data.append(xVel*gain)
    velMsg.data.append(yVel*gain)
    velMsg.data.append(zVel*gain)

    quatWrist = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    quatRotate = [1,0,0,pi]
    quatFid = [wrist_tf.transform.rotation.x, wrist_tf.transform.rotation.y, wrist_tf.transform.rotation.z, wrist_tf.transform.rotation.w]
    quatFid = quaternion_multiply(quatRotate, quatFid)
    quatError = quaternion_multiply(quatWrist, quatFid)

    if quatError[3] <= pi:
        rot_gain = gain
    else: 
        rot_gain = -gain

    
    velMsg.data.append(quatError[1] * rot_gain)
    velMsg.data.append(quatError[2] * rot_gain)
    velMsg.data.append(quatError[3] * rot_gain)
    velocity_pub.publish(velMsg)



if __name__ == "__main__":
    rospy.init_node("VelocityNode")
    rospy.loginfo("Starting VelocityNode.")
    fiducial_sub = rospy.Subscriber("aruco_single/pose", PoseStamped, procFiducial, queue_size=10)
    velocity_pub = rospy.Publisher("eeVel", Float32MultiArray, queue_size=10) 
        
    while not rospy.is_shutdown():
        pass
