#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import websocket
import json

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder', anonymous=True)
        
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        
        self.path = Path()
        self.path.header.frame_id = "odom"

        self.ws = websocket.WebSocket()
        self.ws.connect("ws://your_websocket_server:port")

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path.poses.append(pose)
        self.path.header.stamp = rospy.Time.now()
        
        self.path_pub.publish(self.path)
        
        # Convert Path to JSON
        path_json = self.path_to_json(self.path)
        self.ws.send(json.dumps(path_json))

    def path_to_json(self, path):
        path_dict = {
            "header": {
                "seq": path.header.seq,
                "stamp": {
                    "secs": path.header.stamp.secs,
                    "nsecs": path.header.stamp.nsecs
                },
                "frame_id": path.header.frame_id
            },
            "poses": [
                {
                    "header": {
                        "seq": pose.header.seq,
                        "stamp": {
                            "secs": pose.header.stamp.secs,
                            "nsecs": pose.header.stamp.nsecs
                        },
                        "frame_id": pose.header.frame_id
                    },
                    "pose": {
                        "position": {
                            "x": pose.pose.position.x,
                            "y": pose.pose.position.y,
                            "z": pose.pose.position.z
                        },
                        "orientation": {
                            "x": pose.pose.orientation.x,
                            "y": pose.pose.orientation.y,
                            "z": pose.pose.orientation.z,
                            "w": pose.pose.orientation.w
                        }
                    }
                }
                for pose in path.poses
            ]
        }
        return path_dict

if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass