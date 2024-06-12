#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclRepublisher:
    def __init__(self):
        self.sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback)
        self.pub = rospy.Publisher("/amcl_pose_republished", PoseWithCovarianceStamped, queue_size=10)
        self.latest_pose = None
        self.rate = rospy.Rate(10)  # 10 Hz republishing rate

    def callback(self, msg):
        self.latest_pose = msg

    def publish_pose(self):
        while not rospy.is_shutdown():
            if self.latest_pose is not None:
                self.pub.publish(self.latest_pose)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("amcl_republisher")
    republisher = AmclRepublisher()
    republisher.publish_pose()
