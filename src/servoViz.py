#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np


current_vector = Point()

def vector_visualisation():
    rospy.init_node('vector_visualisation_node', anonymous= True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
    
    #change the topic and message type that is being subscribed to for aruco marker.
    # vector_sub = rospy.Subscriber('vector_data', VectorData, vector_callback)

    rate = rospy.Rate(10)
    # current_vector = None
    # vector_creator(endEffectorVelocity)

    # testing rviz
    current_vector.x = 1
    current_vector.y = 1
    current_vector.z = 1
    w = 1
    x = 0
    y = 0
    z = 0
    

    while not rospy.is_shutdown():
        if current_vector is not None:
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
            end_point.x = current_vector.x
            end_point.y = current_vector.y
            end_point.z = current_vector.z

            marker.points.append(start_point)
            marker.points.append(end_point)
            marker.scale.x = 0.05
            marker.scale.y = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # Set the orientation (quaternion) - Identity quaternion
            marker.pose.orientation.x = x
            marker.pose.orientation.y = y
            marker.pose.orientation.z = z
            marker.pose.orientation.w = w

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
            plane.pose.orientation.x = x
            plane.pose.orientation.y = y
            plane.pose.orientation.z = z
            plane.pose.orientation.w = w

            marker_pub.publish(plane)

        rate.sleep()


def vector_callback(data):
    global endEffectorVelocity
    endEffectorVelocity = data

def vector_creator(EEvelocity):
    global velocity_x
    velocity_x = EEvelocity.x
    global velocity_y
    velocity_y = EEvelocity.y
    global velocity_z
    velocity_z = EEvelocity.z
    global w
    global x 
    global y 
    global z 
    w, x, y, z = euler_to_quaternion(EEvelocity.roll, EEvelocity.pitch, EEvelocity.yaw)

    

def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz
    

if __name__ == '__main__':
    
    try:
        vector_visualisation()
    except rospy.ROSInterruptException:
        pass