#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import tf



if __name__ == "__main__":
    rospy.init_node("add_frame")

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/zed_base_link', rospy.Time(0))
            angles = tf.transformations.euler_from_quaternion(rot, axes='rzyx')
            # print(angles)
            q = tf.transformations.quaternion_from_euler(-angles[2], -angles[1], 0, axes='rxyz')
            yaw = tf.transformations.quaternion_multiply(rot, q)
            check_angles = tf.transformations.euler_from_quaternion(yaw, axes='rzyx')
            # print(check_angles)
            br.sendTransform((0,0,0), q, rospy.Time.now(), "/heading_frame", "/zed_base_link")

            # print("-----------------------")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("lookup failed")
            pass

        rate.sleep()

    
    

    
