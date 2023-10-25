#!/usr/bin/env python
import rospy
import tf2_ros



# get the transformation from source_frame to target_frame.
def get_transform_wrist():
    tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
    tf2_ros.TransformListener(tf_buffer)
    try:
        transformation = tf_buffer.lookup_transform('world', 'wrist_3_link', rospy.Time(0), rospy.Duration(0.1))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s' % 'aruco_marker::link', 'robot::wrist_3_link')

def get_transform_fid():
    tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
    tf2_ros.TransformListener(tf_buffer)
    try:
        transformation = tf_buffer.lookup_transform('map', 'wrist_3_link', rospy.Time(0), rospy.Duration(0.1))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s' % 'aruco_marker::link', 'robot::wrist_3_link')



if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    while not rospy.is_shutdown():
        
        print(get_transform_wrist())   
