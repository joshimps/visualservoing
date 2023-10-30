#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from math import sin, cos
from cv_bridge import CvBridge

"""
process_depth(depth, pixel pose)

Process depth data to determine marker orientation and position.

Parameters:
    depth (depth_image_msg): Depth image message from a depth camera.
    pixel (Pixel): Object with pixel coordinates (u, v) for marker centre.
    pose (PoseStamped): Pose estimate from aruco ros.

Returns:
    Publishes improved pose estimate with time of flight depth data on /aruco_single/pose_rgbd topic 

This function extracts the depth at the specified pixel coordinates, calculates the
marker's depth in meters, and updates the marker's z-coordinate in the pose to
incorporate the depth. The result is published to the 'rgbd_pub' ROS topic.
"""
def process_depth(depth, pixel, pose):

    # Extract Pixels
    u = int(round(pixel.point.x))
    v = int(round(pixel.point.y))
    # Convert depth image to mm
    depthCV = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    # Extract depth value from depth image in metres

    marker_depth = depthCV[v,u]/1000
    
    # Use P3P algorithm from aruco ros node to determine orientation, x and y values
    # replace z value with depth from depth image
    msg = PoseStamped()
    msg.header = pose.header
    msg.pose.orientation = pose.pose.orientation
    msg.pose.position = pose.pose.position
    msg.pose.position.x = marker_depth
    rgbd_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("Aruco_RGBD")
    rospy.loginfo("Starting Aruco_RGBD.")
    # depth image subscriber
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    # marker centrepoint pixel subscriber
    pixel_sub = message_filters.Subscriber("/aruco_single/pixel", PointStamped)
    # parker PnP pose estimate subscriber
    pose_sub = message_filters.Subscriber("/aruco_single/pose", PoseStamped)

    # improved depth pose estimate publisher
    rgbd_pub = rospy.Publisher("/aruco_single/pose_rgbd", PoseStamped, queue_size=5)
    # object to convert ros image messsage to OpenCV image file to extract depth information in mm
    bridge = CvBridge()

    # Subscribe to depth image, pixel centre and pose of detected marker
    sync_sub = message_filters.ApproximateTimeSynchronizer([depth_sub,pixel_sub, pose_sub], 10, 0.5)

    sync_sub.registerCallback(process_depth)    
    while not rospy.is_shutdown():
        pass