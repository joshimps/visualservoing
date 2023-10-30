#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from math import sin, cos
from cv_bridge import CvBridge

bridge = CvBridge()

bridge = CvBridge()

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
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    pixel_sub = message_filters.Subscriber("/aruco_single/pixel", PointStamped)
    pose_sub = message_filters.Subscriber("/aruco_single/pose", PoseStamped)

    rgbd_pub = rospy.Publisher("/aruco_single/pose_rgbd", PoseStamped, queue_size=5)

    # Subscribe to depth image, pixel centre and pose of detected marker
    sync_sub = message_filters.ApproximateTimeSynchronizer([depth_sub,pixel_sub, pose_sub], 10, 0.5)

    sync_sub.registerCallback(process_depth)    
    while not rospy.is_shutdown():
        pass