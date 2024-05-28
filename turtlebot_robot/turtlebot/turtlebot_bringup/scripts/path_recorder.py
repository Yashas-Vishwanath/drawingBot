#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder', anonymous=True)
        
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        
        self.path = Path()
        self.path.header.frame_id = "odom"

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path.poses.append(pose)
        self.path.header.stamp = rospy.Time.now()
        
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
