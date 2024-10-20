import rospy
from visualization_msgs.msg import Marker

class DesiredPathMarker:
    def __init__(self) -> None:
        self.marker_pub = rospy.Publisher('controller/desired_path',Marker,queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.id = 22
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.ns = 'liness'
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.pose.orientation.w = 1.0
    
    def publishMarker(self, pos1, pos2, pos3):
        self.marker.points.clear()
        self.marker.header.stamp.secs = rospy.Time.now().secs
        self.marker.header.stamp.nsecs = rospy.Time.now().nsecs
        self.marker.points.append(pos1)
        self.marker.points.append(pos2)
        self.marker.points.append(pos3)
        self.marker_pub.publish(self.marker)
    
    def setMarkerColor(self, state):
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.2