import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

#PUBLISHERS
def cmd_pub(lin_x:float, lin_y:float, rot_z: float) -> None:
    cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    cmd_vel = Twist()
    cmd_vel.linear.x = lin_x
    cmd_vel.linear.y = lin_y
    cmd_vel.angular.z = rot_z  # Set the angular velocity
    cmd_vel_pub.publish(cmd_vel)

#SUBSCRIBERS
def scan_callback(msg):
    # Just printing the laser scan data for debugging purposes
    ranges = msg.ranges
    rospy.loginfo(f"Received laser scan data: {ranges}")

def odom_callback(msg):
    # Just printing the odometry data for debugging purposes
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    rospy.loginfo(f"Received odometry data - Position: {position}, Orientation: {orientation}")

def move_forward():
    # Initialize the ROS node
    rospy.init_node('move_forward', anonymous=True)

    # Subscribe to laser scan and odometry topics
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Wait for the publisher to be ready
    rospy.sleep(1)

    # Publish the velocity command to make the robot move forward
    cmd_pub(0.2, 0.0, 0.0)  # Linear velocity of 0.2 m/s

    # Wait for the robot to move forward approximately 1 meter
    rospy.sleep(5)

    # Stop the robot by publishing a zero velocity command
    cmd_pub(0.0, 0.0, 0.0)

    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass

