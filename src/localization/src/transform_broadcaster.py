#!/usr/bin/python3

import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class TransformBroadcaster:
    def __init__(self):
        rospy.init_node('transform_broadcaster_node')
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_subscriber = rospy.Subscriber('/leg_odom', PoseWithCovarianceStamped, self.odom_callback)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.yaw_pub = rospy.Publisher('/yaw', Float32, queue_size=10)
        
        self.position = None
        self.orientation = None

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position

    def imu_callback(self, msg):
        self.orientation = msg.orientation
        _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        yaw = yaw * 180 / np.pi
   
        if yaw < 0:
            yaw = 360 + yaw
        yaw_msg = Float32()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)
        

    def publish_transform(self):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "nyk/base_link"

        if self.position is not None and self.orientation is not None:
            transform.transform.translation.x = self.position.x
            transform.transform.translation.y = self.position.y
            transform.transform.translation.z = self.position.z
            transform.transform.rotation = self.orientation

            self.broadcaster.sendTransform(transform)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_transform()
            rate.sleep()

if __name__ == '__main__':
    transform_broadcaster = TransformBroadcaster()
    transform_broadcaster.run()


