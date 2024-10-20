#!/usr/bin/python3

import rospy
import json
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os
from loguru import logger

class DummyPathPublisher:
    def __init__(self, json_file):
        rospy.init_node('dummy_path_publisher')
        self.path_pub = rospy.Publisher('/patrol/path', Path, queue_size=10)
        self.json_file = json_file
        self.path = Path()

        self.load_path_from_json()

    def load_path_from_json(self):
        if not os.path.exists(self.json_file):
            logger.error(f"JSON file not found: {self.json_file}")
            return

        try:
            with open(self.json_file, 'r') as f:
                data = json.load(f)
                self.path.header.stamp = rospy.Time.now()
                self.path.header.frame_id = "map"
                
                for point in data['path']:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = point['x']
                    pose.pose.position.y = point['y']
                    pose.pose.position.z = point['z']
                    pose.pose.orientation.w = 1.0  
                    self.path.poses.append(pose)

                logger.info("Path loaded successfully from JSON.")
        except json.JSONDecodeError:
            logger.error(f"Failed to decode JSON from {self.json_file}")
        except Exception as e:
            logger.error(f"Error loading path from JSON: {e}")

    def publish_path(self):
        if self.path.poses:
            self.path_pub.publish(self.path)
            logger.info("Published path to /patrol/path.")

    def run(self):
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            self.publish_path()
            rate.sleep()


if __name__ == "__main__":
    json_file_path = "../config/path.json"
    try:
        dummy_path_publisher = DummyPathPublisher(json_file_path)
        dummy_path_publisher.run()
    except rospy.ROSInterruptException:
        logger.error("ROS Interrupt Exception")
