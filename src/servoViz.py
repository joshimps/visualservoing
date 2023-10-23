#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


counter = 0

def vector_callback(data):

  marker = Marker()
  marker.header.frame_id = "map"
  marker.header.stamp = rospy.Time.now()
  marker.type = Marker.ARROW
  marker.action = Marker.ADD
  marker.ns = "vector_arrow"
  start_point = Point()
  start_point.x = 0.0
  start_point.y = 0.0
  start_point.z = 0.5
  end_point = Point()

  end_point.x = data.pose.position.x
  end_point.y = data.pose.position.y
  end_point.z = data.pose.position.z + 0.5

  marker.points.append(start_point)
  marker.points.append(end_point)
  marker.scale.x = 0.05
  marker.scale.y = 0.1
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 0.0
  marker.color.b = 0.0

  # Set the orientation (quaternion) - Identity quaternion
  marker.pose.orientation.x = 0
  marker.pose.orientation.y = 0
  marker.pose.orientation.z = 0
  marker.pose.orientation.w = 1

  marker_pub.publish(marker)

  # Plane Marker
  plane = Marker()
  plane.header.frame_id = "map"
  plane.header.stamp = rospy.Time.now()
  plane.type = Marker.MESH_RESOURCE
  plane.mesh_resource = "package://visual_servoing/src/include/airplane.stl"
  plane.action = Marker.ADD
  plane.ns = "airplane"

  plane.scale.x = 1.0
  plane.scale.y = 1.0
  plane.scale.z = 1.0
  plane.color.a = 1.0
  plane.color.r = 1.0
  plane.color.g = 1.0
  plane.color.b = 1.0

  # set marker position
  plane.pose.position.x = 0.2
  plane.pose.position.y = -0.2
  plane.pose.position.z = 0.1

  plane.pose.orientation.x = round(data.pose.orientation.x,3)
  plane.pose.orientation.y = round(data.pose.orientation.y,3)
  plane.pose.orientation.z = round(data.pose.orientation.z,3)
  plane.pose.orientation.w = round(data.pose.orientation.w,3)

  marker_pub.publish(plane)

if __name__ == '__main__': 
    
    rospy.init_node('vector_visualisation_node', anonymous= True)

    mode = rospy.get_param("~rgbd_mode", default=False)

    if mode:
        rospy.loginfo("RGBD Mode: %s", mode)
        rgbd_sub = rospy.Subscriber('/aruco_single/pose_rgbd', PoseStamped, vector_callback)
    else:
        rospy.loginfo("RGBD Mode: %s", mode)
        velocity_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, vector_callback)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=3)
    # velocity_sub = rospy.Subscriber('eeVel', Float32MultiArray,  vector_callback)
    
    while not rospy.is_shutdown():
        pass