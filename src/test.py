#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray 

if __name__ == "__main__":
    rospy.init_node("spinner")
    rospy.loginfo("Starting spinner.")
    message_pub = rospy.Publisher("joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
    msg = Float64MultiArray()
    msg.data = [0,0,0,0,0,0]

    while not rospy.is_shutdown():
        message_pub.publish(msg)
        pass
        