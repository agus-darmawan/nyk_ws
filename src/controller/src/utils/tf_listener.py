import rospy

import tf.transformations as tf
import tf2_ros

from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped

class TFListener:
    def __init__(self):
        self.pos = Point()
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.yaw = 0.0
        self.is_tf_available = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listen = tf2_ros.TransformListener(self.tf_buffer)
    
    def listenTF(self):
        try:
            trans : TransformStamped = self.tf_buffer.lookup_transform('map','nyk/base_link',rospy.Time())
            self.is_tf_available = True
        except ():
            self.is_tf_available = False
            return False,0,0,0,0,0,0

        quaternion = (
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)

        euler = tf.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.pos.x = trans.transform.translation.x
        self.pos.y = trans.transform.translation.y
        self.yaw = yaw 
        