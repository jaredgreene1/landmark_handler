#!/usr/bin/env python  
import rospy

import math
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf_transform')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            baselink2odom = tfBuffer.lookup_transform("baselink", "odom", rospy.Time())
            odom2map = tfBuffer.lookup_transform("odom", "map", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print trans.transform
        rate.sleep()
