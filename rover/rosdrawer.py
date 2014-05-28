
import pygame.color as color
import collections
from visualization_msgs.msg import Marker
import rospy


class Drawer(object):

    def __init__(self, problem, colors):
        rospy.init_node("rover_drawer", anonymous=True)
        self.pub = rospy.Publisher("visualization_marker", Marker)

        self.problem = problem
        self.colors = colors
        self.color_dict = color.THECOLORS

        self.clear_all().update()
        self.markers = collections.deque(list(), 100)

    def draw_evader(self, e_x, e_y):
        return self

    def draw_coverage(self):
        return self

    def add_coverage(self, quad):
        return self

    def draw_quad(self, quad):

        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(1)
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 10
            marker.scale.y = 10
            marker.scale.z = 10
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = quad.get_x()
            marker.pose.position.y = quad.get_y()
            marker.pose.position.z = quad.get_z()
            marker.id = hash(quad)
            self.markers.append(marker)

        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(1)
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 2 * quad.get_sensor_radius()
            marker.scale.y = 2 * quad.get_sensor_radius()
            marker.scale.z = 1
            marker.color.a = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = quad.get_x()
            marker.pose.position.y = quad.get_y()
            marker.pose.position.z = 0
            marker.id = self.problem.num_quads + hash(quad) + 1
            self.markers.append(marker)

        return self

    def draw_risk_grid(self, risk_grid):
        for i, (_, _, r_point) in enumerate(risk_grid.get_risk_points()):
            if not rospy.is_shutdown():
                marker = Marker()
                marker.header.frame_id = "/my_frame"
                marker.lifetime = rospy.Duration(1)
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.scale.x = 20
                marker.scale.y = 20
                marker.scale.z = 40
                marker.color.a = 1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = r_point.get_x()
                marker.pose.position.y = r_point.get_y()
                marker.pose.position.z = 0
                marker.id = i
                self.markers.append(marker)
        return self

    def update(self):
        for marker in self.markers:
            self.pub.publish(marker)

        return None

    def update_with_frame(self, frame):
        return self

    def get_key_pressed(self):
        return self

    def clear_all(self):
        self.markers = collections.deque(list(), 100)
        return self

    def can_play(self):
        return False
