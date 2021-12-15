#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg



CAMERA_TO_RIGHT_ARM = (0., 0., 0.)
CAMERA_TO_LEFT_ARM = (0., 0., 0.)

if __name__ == '__main__':
    rospy.init_node('static_pub')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    static_rightTransformStamped = geometry_msgs.msg.TransformStamped()
    static_rightTransformStamped.header.stamp = rospy.Time.now()
    static_rightTransformStamped.header.frame_id = "camera"
    static_rightTransformStamped.child_frame_id = "right_arm"
    static_rightTransformStamped.transform.translation.x = CAMERA_TO_RIGHT_ARM[0]
    static_rightTransformStamped.transform.translation.y = CAMERA_TO_RIGHT_ARM[1]
    static_rightTransformStamped.transform.translation.z = CAMERA_TO_RIGHT_ARM[2]
    static_rightTransformStamped.transform.rotation.x = 0
    static_rightTransformStamped.transform.rotation.y = 0
    static_rightTransformStamped.transform.rotation.z = 0
    static_rightTransformStamped.transform.rotation.w = 1

    static_leftTransformStamped = geometry_msgs.msg.TransformStamped()
    static_leftTransformStamped.header.stamp = rospy.Time.now()
    static_leftTransformStamped.header.frame_id = "camera"
    static_leftTransformStamped.child_frame_id = "left_arm"
    static_leftTransformStamped.transform.translation.x = CAMERA_TO_LEFT_ARM[0]
    static_leftTransformStamped.transform.translation.y = CAMERA_TO_LEFT_ARM[1]
    static_leftTransformStamped.transform.translation.z = CAMERA_TO_LEFT_ARM[2]
    static_leftTransformStamped.transform.rotation.x = 0
    static_leftTransformStamped.transform.rotation.y = 0
    static_leftTransformStamped.transform.rotation.z = 0
    static_leftTransformStamped.transform.rotation.w = 1

    broadcaster.sendTransform(static_rightTransformStamped)
    broadcaster.sendTransform(static_leftTransformStamped)
    rospy.spin()