#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()  # defining the way we can allocate the values
move.linear.x = 0.5  # allocating the values in x direction - linear
move.angular.z = 0.5  # allocating the values in z direction - angular

# Set the duration for which the robot should move (in seconds)
duration = 2.0  # Adjust as needed

# Initialize a timer
start_time = rospy.Time.now()

while not rospy.is_shutdown():
    # Check if the elapsed time exceeds the duration
    if (rospy.Time.now() - start_time).to_sec() < duration:
        # Publish the velocity command
        pub.publish(move)
    else:
        # Stop the robot by publishing zero velocity
        pub.publish(Twist())
        break  # Exit the loop

    rate.sleep()

