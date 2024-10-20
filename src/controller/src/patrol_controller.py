#!/usr/bin/python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from utils.path_following import PathFollowing
from loguru import logger
import sys

class PatrolNode:
    def __init__(self):
        rospy.init_node('patrol_controller_node')
        logger.remove(0)
        logger.add(sys.stderr, format = "<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.patrol_controller = PathFollowing()
        self.path_subscriber = rospy.Subscriber('/patrol/path', Path, self.path_callback)
        self.motion_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def path_callback(self, msg: Path):
        self.patrol_controller.set_path(msg)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            twist = Twist()
            if len (self.patrol_controller.main_path.poses) == 0:
                logger.warning("No patrol path received")
                rate.sleep()
                continue
            if not self.patrol_controller.is_arrived_last_wp():
                angle_error = self.patrol_controller.update()
                twist.linear.x = 0.5
                twist.angular.z = angle_error
            else:
                logger.info("Patrol completed")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.motion_publisher.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    patrol_node = PatrolNode()
    patrol_node.run()

