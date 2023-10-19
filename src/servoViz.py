#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import math
import time
from tf.transformations import quaternion_from_euler
# from scipy.spatial.transform import Rotation as R
import numpy as np
from tf.transformations import euler_from_quaternion

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
  start_point.z = 0.0
  end_point = Point()
  end_point.x = data.data[0] * 5
  end_point.y = data.data[1] * 10
  end_point.z = data.data[2] * 10

  rospy.loginfo(end_point)
  # end_point.x = data.pose.position.z
  # end_point.y = -data.pose.position.x
  # end_point.z = -data.pose.position.y

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

  # Set the orientation (quaternion) - Identity quaternion
  # w, x, y, z = get_quaternion_from_euler(data.data[3]*10, data.data[4]*10, data.data[5]*10)
  # w, x, y, z = get_quaternion_from_euler(d, data.pose.orientation.y, data.pose.orientation.z)
  angular_velocity = [data.data[3],data.data[4], data.data[5]]
  rospy.loginfo('Original:')
  rospy.loginfo(angular_velocity)
  w, x, y, z = quaternion_from_euler(data.data[3],data.data[4], data.data[5])
  roll, pitch, yaw = euler_from_quaternion([w,x,y,z])

  rospy.loginfo('Quaternion:')
  rospy.loginfo(quaternion_from_euler(data.data[3],data.data[4], data.data[5]))

  # reversed_velocity = [roll,pitch,yaw]
  # rospy.loginfo('Reversed Calculated RPY:')
  # rospy.loginfo(euler_from_quaternion([w,x,y,z]))

  global counter 

  if counter == 0:
     x = 0
     y = 0
     z = 0
     w = 1
     counter = 1

  plane.pose.orientation.x = x
  plane.pose.orientation.y = y
  plane.pose.orientation.z = z  
  plane.pose.orientation.w = w

  marker_pub.publish(plane)



def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

# def euler_from_quaternion(x, y, z, w):
#         """
#         Convert a quaternion into euler angles (roll, pitch, yaw)
#         roll is rotation around x in radians (counterclockwise)
#         pitch is rotation around y in radians (counterclockwise)
#         yaw is rotation around z in radians (counterclockwise)
#         """
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll_x = math.atan2(t0, t1)
     
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch_y = math.asin(t2)
     
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw_z = math.atan2(t3, t4)
     
#         return roll_x, pitch_y, yaw_z # in radians

# def euler_to_quaternion(roll, pitch, yaw):
#     cy = np.cos(yaw * 0.5)
#     sy = np.sin(yaw * 0.5)
#     cp = np.cos(pitch * 0.5)
#     sp = np.sin(pitch * 0.5)
#     cr = np.cos(roll * 0.5)
#     sr = np.sin(roll * 0.5)

#     qw = cr * cp * cy + sr * sp * sy
#     qx = sr * cp * cy - cr * sp * sy
#     qy = cr * sp * cy + sr * cp * sy
#     qz = cr * cp * sy - sr * sp * cy

#     return qw, qx, qy, qz
    

if __name__ == '__main__':
    
    
    rospy.init_node('vector_visualisation_node', anonymous= True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=2)
    velocity_sub = rospy.Subscriber('eeVel', Float32MultiArray,  vector_callback)
    # velocity_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, vector_callback)
    rospy.spin()