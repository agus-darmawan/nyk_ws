from loguru import logger
from nav_msgs.msg import Path
from utils.utility import get_angle, rad2deg, distance_line_to_point
from utils.fuzzy_pid import FuzzyPID
from utils.tf_listener import TFListener
from utils.desired_path_marker import DesiredPathMarker
from geometry_msgs.msg import Point


class PathFollowing:
    def __init__(self) -> None:
        self.pid_angle = FuzzyPID()
        self.pid_angle.p = [0.02, 0.03]
        self.pid_angle.i = [0.00, 0.00] 


        self.pid_distance = FuzzyPID()
        self.pid_distance.p = [0.4, 0.6]
        self.pid_distance.upper_bound = 1.0
        self.pid_distance.lower_bound = -1.0

        self.path_angle_limit = 60
        self.path_keep_distance = 0.5

        self.angle_error = 0.0
        self.distance_error = 0.0

        self.main_path = Path()
        self.path_idx = 1
        self.desired_path_marker = DesiredPathMarker()
        self.tf_listener = TFListener()

    def clamp_heading(self, heading):
        """Clamp heading to the range [-180, 180]."""
        if heading > 180:
            heading -= 360
        elif heading < -180:
            heading += 360
        return heading

    def calculate_angle_error(self, pos1: Point, pos2: Point):
        """Calculate the angle error between two points."""
        angle_path = get_angle(pos1, pos2)
        self.angle_error = rad2deg(angle_path - self.tf_listener.yaw)
        self.angle_error = self.clamp_heading(self.angle_error)

    def calculate_distance_error(self):
        """Calculate the distance error from the path."""
        self.distance_error = distance_line_to_point(
            self.main_path.poses[self.path_idx - 1].pose.position,
            self.main_path.poses[self.path_idx].pose.position,
            self.tf_listener.pos
        )

        if abs(self.distance_error) > 4:
            self.distance_error = 0
            angle_to_first_wp = get_angle(self.tf_listener.pos, self.main_path.poses[self.path_idx - 1].pose.position)
            self.angle_error = rad2deg(angle_to_first_wp - self.tf_listener.yaw)
            self.angle_error = self.clamp_heading(self.angle_error)

    def is_arrived_last_wp(self):
        """Check if the last waypoint has been reached."""
        return self.path_idx >= len(self.main_path.poses) - 1 and self.is_arrived_path()

    def get_path_angle(self):
        """Get the angle of the current path segment."""
        if len(self.main_path.poses) <= 1:
            return

        angle_path = get_angle(
            self.main_path.poses[self.path_idx - 1].pose.position,
            self.main_path.poses[self.path_idx].pose.position
        )
        return angle_path

    def is_arrived_path(self):
        """Check if the current path point has been reached."""
        if len(self.main_path.poses) - 1 < self.path_idx:
            return True

        controlled_path_point = self.main_path.poses[self.path_idx].pose.position
        controlled_path_point_before = self.main_path.poses[self.path_idx - 1].pose.position
        
        angle = rad2deg(get_angle(self.tf_listener.pos, controlled_path_point) - 
                        get_angle(controlled_path_point_before, controlled_path_point))

        delta_x = self.tf_listener.pos.x - controlled_path_point.x
        delta_y = self.tf_listener.pos.y - controlled_path_point.y

        angle = abs(angle) if abs(angle) < 180 else 360 - abs(angle)
        
        return not (abs(angle) < self.path_angle_limit and (delta_x ** 2 + delta_y ** 2) > (self.path_keep_distance ** 2))

    def update(self):
        """Update the path following state and compute errors."""
        self.tf_listener.listenTF()
        logger.info("==========PATH FOLLOWING==========")
        logger.info(f"Len Waypoint : {len(self.main_path.poses)}")
        
        if not self.tf_listener.is_tf_available:
            logger.error("No TF Available")
            return 0
        
        if self.is_arrived_last_wp():
            logger.info("Arrived Last WP")
            return 0
        
        if len(self.main_path.poses) < self.path_idx + 1:
            logger.error("Not Enough Waypoint")
            return 0

        if self.is_arrived_path():
            logger.info(f"Arrived Path {self.path_idx}")
            self.path_idx += 1
        
        self.desired_path_marker.setMarkerColor()
        self.desired_path_marker.publishMarker(
            self.tf_listener.pos,
            self.main_path.poses[self.path_idx].pose.position,
            self.main_path.poses[self.path_idx - 1].pose.position
        )
        
        self.calculate_angle_error(
            self.main_path.poses[self.path_idx - 1].pose.position,
            self.main_path.poses[self.path_idx].pose.position
        )
        self.calculate_distance_error()

        angle_error_pid = self.pid_angle.update(self.angle_error)
        distance_error_pid = self.pid_distance.update(self.distance_error)

        logger.info(f"Waypoint Number : {self.path_idx}")
        logger.info(f"==========REAL ERROR PF==========")
        logger.info(f"Angle Error     : {self.angle_error}")
        logger.info(f"Distance Error  : {self.distance_error}")
        logger.info(f"Overall Error   : {self.angle_error + self.distance_error}")
        logger.info(f"==========PID ERROR PF==========")
        logger.info(f"Angle PID       : {angle_error_pid}")
        logger.info(f"Distance PID    : {distance_error_pid}")
        logger.info(f"Overall PID     : {angle_error_pid + distance_error_pid}")

        return angle_error_pid + distance_error_pid

    def set_path(self, path: Path):
        """Set the path for following."""
        logger.info("Path Successfully Set")
        self.main_path = path
        self.path_idx = 1
