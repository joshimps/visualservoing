#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from math import sin, cos

HEIGHT = 480
WIDTH = 640
VFOV = 42
HFOV = 69
ANGLE_INC_H = HFOV/WIDTH  
MIN_ANGLE_H = (HFOV/2)
ANGLE_INC_V = VFOV/HEIGHT  
MIN_ANGLE_V = -(VFOV/2)

def process_depth(depth, pixel, pose):
    u = pixel.point.x
    theta = MIN_ANGLE_H - ANGLE_INC_H * u
    v = pixel.point.y
    gamma = MIN_ANGLE_V + ANGLE_INC_V * v
    index = u * depth.step + v
    marker_depth = depth.data[index]
    x = marker_depth * sin(theta)
    y = marker_depth * sin(gamma)
    z = marker_depth * cos(gamma)
    
    msg = PoseStamped()
    msg.header = pose.header
    msg.pose.orientation = pose.pose.orientation
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    rgbd_pub.publish(msg)



if __name__ == "__main__":
    rospy.init_node("Aruco_RGBD")
    rospy.loginfo("Starting Aruco_RGBD.")
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    pixel_sub = message_filters.Subscriber("/aruco_singe/pixel", PointStamped)
    pose_sub = message_filters.Subscriber("/aruco_single/pose", PoseStamped)

    rgbd_pub = rospy.Publisher("/aruco_single/pose_rgbd", PoseStamped, queue_size=5)
    
    sync_sub = message_filters.ApproximateTimeSynchronizer([depth_sub,pixel_sub, pose_sub], 5, 0.2)
    sync_sub.registerCallback(process_depth)    
    while not rospy.is_shutdown():
        pass
