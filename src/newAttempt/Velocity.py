#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np

## CFG ##
vel_gain = 0.02
rot_gain = 30
refQuat = PoseStamped()
refQuat.pose.orientation.x = 0
refQuat.pose.orientation.y = 0.5
refQuat.pose.orientation.z = 0
refQuat.pose.orientation.w = 0

def get_transform_wrist():
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
    tf2_ros.TransformListener(tf_buffer)
    try:
        transformation = tf_buffer.lookup_transform('base_link', 'wrist_3_link', rospy.Time(0), rospy.Duration(0.1))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s' % 'base', 'wrist')

def angularVelfromQuat(qsource, qtarget, timestep):
    return (2 / timestep) * np.array([
        qsource.w*qtarget.x - qsource.x*qtarget.w - qsource.y*qtarget.z + qsource.z*qtarget.y,
        qsource.w*qtarget.y + qsource.x*qtarget.z - qsource.y*qtarget.w - qsource.z*qtarget.x,
        qsource.w*qtarget.z - qsource.x*qtarget.y + qsource.y*qtarget.x - qsource.z*qtarget.w])

def procFiducial(msg):
    velMsg = Float32MultiArray()
    velMsg.data.append(msg.pose.position.x * vel_gain)
    velMsg.data.append(msg.pose.position.y * vel_gain)
<<<<<<< HEAD
    velMsg.data.append(msg.pose.position.z * vel_gain)
=======
    velMsg.data.append((msg.pose.position.z-0.5) * vel_gain)
>>>>>>> f0a273cab3d6dd681d5f5137f4e4148566ae6669
    qSource = msg.pose.orientation
    qTarget = refQuat.pose.orientation
    angularVel = angularVelfromQuat(qSource, qTarget, rot_gain)
    velMsg.data.append(angularVel[0])
    velMsg.data.append(angularVel[1])
    velMsg.data.append(angularVel[2])
    velocity_pub.publish(velMsg)
    print(velMsg.data)



    

if __name__ == "__main__":
    rospy.init_node("VelocityNode")
    rospy.loginfo("Starting VelocityNode.")
    fiducial_sub = rospy.Subscriber("aruco_single/pose", PoseStamped, procFiducial, queue_size=10)
    velocity_pub = rospy.Publisher("eeVel", Float32MultiArray, queue_size=10) 
        
    while not rospy.is_shutdown():
        pass
