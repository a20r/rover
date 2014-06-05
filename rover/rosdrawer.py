
import pygame.color as color
import collections
from visualization_msgs.msg import Marker
import rospy
import random
import math


class Drawer(object):

    def __init__(self, problem, colors):
        rospy.init_node("rover_drawer", anonymous=True)
        self.pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=1000
        )

        self.problem = problem
        self.colors = colors
        self.color_dict = color.THECOLORS

        self.clear_all().update()
        self.markers = collections.deque(list(), 100)
        self.duration = 3

    def hash32(self, value):
        return hash(value) & 0xffffffff

    def draw_evader(self, e_x, e_y):
        return self

    def draw_coverage(self):
        return self

    def add_coverage(self, quad):
        return self

    def angle2quat(self, bank, heading, attitude):
        #Assuming the angles are in radians.
        c1 = math.cos(heading/2)
        s1 = math.sin(heading/2)
        c2 = math.cos(attitude/2)
        s2 = math.sin(attitude/2)
        c3 = math.cos(bank/2)
        s3 = math.sin(bank/2)
        c1c2 = c1*c2
        s1s2 = s1*s2
        w = c1c2*c3 - s1s2*s3
        x = c1c2*s3 + s1s2*c3
        y = s1*c2*c3 + c1*s2*s3
        z = c1*s2*c3 - s1*c2*s3
        n = math.sqrt(pow(w, 2) + pow(w, 2) + pow(w, 2) + pow(w, 2))
        w_n = w / n
        x_n = x / n
        y_n = y / n
        z_n = z / n
        return (w_n, x_n, y_n, z_n)


    def draw_quad(self, quad):

        quad_id = self.hash32(quad)

        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(self.duration)
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
            marker.id = quad_id
            self.markers.append(marker)

        rotation = self.angle2quat(
            0, 0, quad.get_orientation()
        )

        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(self.duration)
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 2 * quad.get_ellipse_major()
            marker.scale.y = 2 * quad.get_ellipse_minor()
            marker.scale.z = 1
            marker.color.a = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = math.cos(
                math.radians(quad.get_orientation() / 2)
            )
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = math.sin(
                math.radians(quad.get_orientation() / 2)
            )
            marker.pose.position.x = quad.get_ellipse_center()[0]
            marker.pose.position.y = quad.get_ellipse_center()[1]
            marker.pose.position.z = 0
            marker.id = quad_id + self.problem.num_quads
            self.markers.append(marker)

        return self

    def draw_risk_grid(self, risk_grid):
        for i, (_, _, r_point) in enumerate(risk_grid.get_risk_points()):
            if not rospy.is_shutdown():
                marker = Marker()
                marker.header.frame_id = "/my_frame"
                marker.lifetime = rospy.Duration(self.duration)
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
