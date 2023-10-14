#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Point as VectorData
import numpy as np


current_vector = Point()

def vector_visualisation():
    rospy.init_node('vector_visualisation_node', anonymous= True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
    
    #change the topic and message type that is being subscribed to for aruco marker.
    vector_sub = rospy.Subscriber('vector_data', VectorData, vector_callback)
    rate = rospy.Rate(10)
    # current_vector = None

    # testing rviz
    current_vector.x = 1
    current_vector.y = 1
    current_vector.z = 1
    

    while not rospy.is_shutdown():
        if current_vector is not None:
            marker = Marker()
            marker.header.frame_id = "base_link"
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
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker_pub.publish(marker)

        rate.sleep()


def vector_callback(data):
    global endEffectorVelocity
    endEffectorVelocity = data

# def vector_creator(EEvelocity):
#     global velocity_x = EEvelocity.x
#     global velocity_y = EEvelocity.y
#     global velocity_z = EEvelocity.z
    

    

if __name__ == '__main__':
    print("we made it to in the script")
    try:
        vector_visualisation()
    except rospy.ROSInterruptException:
        pass