
import collections
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import matplotlib.cm as cm
import matplotlib.colors as colors
import rospy
import math


class Drawer(object):

    def __init__(self, problem):
        self.pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=1000
        )

        self.problem = problem

        self.clear_all().update()
        self.markers = collections.deque(list(), 1000)
        self.duration = 3
        norm = colors.Normalize(vmin=0, vmax=1)
        self.sm = cm.ScalarMappable(norm=norm)
        self.sm.set_cmap("jet")

    def hash32(self, value):
        return hash(value) & 0xffffffff

    def draw_line(self, quad, x, y, z, hash_val):
        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(self.duration)
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.scale.x = 4
            marker.color.b = 1.0
            marker.color.a = 1.0

            p1 = Point()
            p1.x = quad.get_x()
            p1.y = quad.get_y()
            p1.z = quad.get_z()

            p2 = Point()
            p2.x = x
            p2.y = y
            p2.z = z

            marker.points.append(p1)
            marker.points.append(p2)
            marker.pose.orientation.w = 1.0
            marker.id = hash_val
            self.markers.append(marker)

        return self

    def draw_quad_sensor_coverage(self, quad):
        quad_id = self.hash32(quad)
        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(self.duration)
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 2 * quad.get_ellipse_major()
            marker.scale.y = 2 * quad.get_ellipse_minor()
            marker.scale.z = 1
            marker.color.a = 0.6
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
            marker.id = quad_id + 100
            self.markers.append(marker)

        return self

    def draw_quad_base(self, quad):
        quad_id = self.hash32(quad)
        if not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(self.duration)
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = "package://hector_quadrotor_description/"\
                + "meshes/quadrotor/quadrotor_base.dae"
            marker.action = marker.ADD
            marker.scale.x = 80
            marker.scale.y = 80
            marker.scale.z = 80
            marker.mesh_use_embedded_materials = True

            marker.pose.orientation.w = math.cos(
                math.radians(quad.get_orientation() / 2)
            )
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = math.sin(
                math.radians(quad.get_orientation() / 2)
            )

            marker.pose.position.x = quad.get_x()
            marker.pose.position.y = quad.get_y()
            marker.pose.position.z = quad.get_z()
            marker.id = quad_id
            self.markers.append(marker)

    def draw_quad(self, quad):
        self.draw_quad_base(quad)

        self.draw_quad_sensor_coverage(quad)
        self.draw_line(
            quad, quad.get_ellipse_center()[0],
            quad.get_ellipse_center()[1], 0,
            -self.hash32(quad)
        )

        return self

    def draw_risk_grid(self, risk_grid):
        if not rospy.is_shutdown():
            p_list = list()
            c_list = list()
            x_gen = xrange(0, self.problem.width, 10)
            y_gen = xrange(0, self.problem.height, 10)

            for i in x_gen:
                for j in y_gen:
                    risk = risk_grid[i, j]
                    pnt = Point(i, j, -10)
                    r, g, b, a = self.sm.to_rgba(risk)
                    clr = ColorRGBA(r, g, b, a)
                    p_list.append(pnt)
                    c_list.append(clr)

            marker = Marker()
            marker.header.frame_id = "/my_frame"
            marker.lifetime = rospy.Duration(10000000)
            marker.type = marker.POINTS
            marker.scale.x = 10
            marker.scale.y = 10
            marker.action = marker.ADD
            marker.points = p_list
            marker.colors = c_list
            marker.id = 1
            self.pub.publish(marker)
        return self

    def update(self):
        for marker in self.markers:
            self.pub.publish(marker)

        return None

    def clear_all(self):
        self.markers = collections.deque(list(), 100)
        return self
