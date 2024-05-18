#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def publish_transform():
    rospy.init_node('transform_publisher')
    listener = tf.TransformListener()
    pub = rospy.Publisher('/accurate_odom', Odometry, queue_size=10)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = trans[0]
            odom.pose.pose.position.y = trans[1]
            odom.pose.pose.position.z = trans[2]
            odom.pose.pose.orientation.x = rot[0]
            odom.pose.pose.orientation.y = rot[1]
            odom.pose.pose.orientation.z = rot[2]
            odom.pose.pose.orientation.w = rot[3]

            pub.publish(odom)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
