from geometry_msgs.msg import Point
import numpy as np

def get_vector_angle(cp: Point) -> float:
    return np.arctan2(cp.y, cp.x) - np.pi / 2

def get_angle(pos1: Point, pos2: Point):
    return np.arctan2(-pos1.y + pos2.y, -pos1.x + pos2.x)

def rad2deg(anglee: float):
    return anglee * 180 / np.pi

def distance_line_to_point(init: Point, dest: Point, curr: Point):
    y2_minus_y1 = dest.y - init.y
    x2_minus_x1 = dest.x - init.x
    return ((y2_minus_y1 * curr.x - x2_minus_x1 * curr.y + dest.x * init.y - dest.y * init.x)
            / np.sqrt(y2_minus_y1 * y2_minus_y1 + x2_minus_x1 * x2_minus_x1))
