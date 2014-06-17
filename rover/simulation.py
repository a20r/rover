
import planner
import time
import stats
import json
import plot
import random
import math
import quadcopter
import rospy
import tf
from geometry_msgs.msg import Pose


class Simulation(object):

    def __init__(self, problem, risk_grid, **kwargs):
        self.problem = problem
        self.planner_obj = kwargs.get("algorithm", planner.PlannerGaussian)
        self.practical = kwargs.get("practical", False)
        names = kwargs.get("names", dict())
        self.pubs = self.init_pubs(names)
        self.quad_list = self.init_quads()
        self.pl = self.planner_obj(problem, risk_grid, self.quad_list)
        self.drawer = kwargs.get("drawer", None)
        self.show_time_grid = kwargs.get("show_time_grid", True)
        self.risk_grid = risk_grid
        self.mca = stats.MonteCarloArea(problem, 1000)
        self.sqa = stats.SensorQualityAverage(self.pl)
        self.ra = stats.RiskAverage(self.pl)
        self.init_stats(**kwargs)

        self.listener = tf.TransformListener()

    def init_pubs(self, names):
        pub_dict = dict()
        for name, topic_name in names.iteritems():
            pub_dict[name] = rospy.Publisher(
                topic_name, Pose, queue_size=10
            )
        return pub_dict

    def init_stats(self, **kwargs):
        if not kwargs.get("out_file", None) is None:
            self.out_file = open(kwargs.get("out_file", None), "w")
        else:
            self.out_file = None

        if not kwargs.get("position_file", None) is None:
            self.position_file = open(kwargs.get("position_file", None), "w")
        else:
            self.position_file = None

        if self.show_time_grid:
            self.t_plotter = plot.TimeGridPlotter(self.problem.grid)

    def init_quads(self):
        quad_list = list()
        if self.practical:
            # Gets quad positions from Vicon
            for name, _ in self.pubs.iteritems():
                quad = quadcopter.Quadcopter(self.problem, name)
                x, y, z, b = self.get_actual_configuration(quad)
                quad.set_position(x, y, z)
                quad.set_orientation(b)
                quad_list.append(quad)
        else:
            # Puts the quads into a grid if it is not a practical experiment
            init_length = math.ceil(math.sqrt(self.problem.num_quads))
            quad_spacing = 20
            for i in xrange(self.problem.num_quads):
                down = int(i // init_length)
                accross = int(i % init_length)

                s_x, s_y = (
                    100 + quad_spacing * self.problem.quad_size * accross,
                    100 + quad_spacing * self.problem.quad_size * down
                )

                quad = quadcopter.Quadcopter(self.problem)
                quad.set_position(s_x, s_y, self.problem.min_height)
                quad.set_orientation(0)
                quad.set_camera_angle(self.problem.initial_camera_angle)
                quad_list.append(quad)

        return quad_list

    def publish_configuration(self, quad, heading, beta, phi):
        if self.practical:
            pose = Pose()
            pose.orientation.w = math.cos(math.radians(beta / 2))
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = math.sin(math.radians(beta / 2))

            pose.position.x = (
                quad.x + heading.get_x() *
                self.problem.step_size -
                self.problem.width / 2
            ) / 100

            pose.position.y = (
                quad.y + heading.get_y() *
                self.problem.step_size -
                self.problem.height / 2
            ) / 100

            pose.position.z = (
                quad.z + heading.get_z() *
                self.problem.step_size * 3
            ) / 100

            self.pubs[quad.get_name()].publish(pose)
            #else:
            quad.set_position(
                quad.x + heading.get_x() * self.problem.step_size,
                quad.y + heading.get_y() * self.problem.step_size,
                quad.z + heading.get_z() * self.problem.step_size
            )
        return self

    def get_actual_configuration(self, quad):
        if self.practical:
            now = rospy.Time.now()
            tf_world = "/world"
            our_obj = "/" + quad.get_name() + "/base_link",

            listener.waitForTransform(
                tf_world, our_obj,
                now, rospy.Duration(0.1)
            )

            trans, rot = listener.lookupTransform(
                tf_world, our_obj, now
            )

            _, _, beta = tf.transforms.euler_from_quaternion(rot)

            return (
                trans[0],
                trans[1],
                trans[2],
                math.degrees(beta)
            )

        else:
            return quad.x, quad.y, quad.z, quad.beta

    def run(self):
        for i in xrange(self.problem.num_steps):
            for quad in self.quad_list:
                try:
                    rpx, rpy, rpz, rb = self.get_actual_configuration(quad)
                    quad.set_position(rpx, rpy, rpz)
                    quad.set_orientation(rb)
                    self.problem.grid.update_grid(quad)
                    heading, beta, phi = self.pl.get_next_configuration(quad)
                    self.publish_configuration(quad, heading, beta, phi)
                except tf.Exception as e:
                    print e
                    self.publish_configuration(
                        quad, 0, 0, quad.beta, quad.phi
                    )

            if self.show_time_grid:
                self.t_plotter.update()

            if not self.drawer is None:
                self.drawer.clear_all()

                self.drawer.draw_risk_grid(self.risk_grid)

                for quad in self.quad_list:
                    self.drawer.draw_quad(quad)

                frame = self.drawer.update()

            self.mca.update_average_efficiency(self.quad_list)
            self.sqa.update_average_sq(self.quad_list)
            self.ra.update_average_risk(self.quad_list)
            self.write_results(self.quad_list, i)

    def write_results(self, quads, i):
        if not self.out_file is None:
            self.out_file.write(
                str(self.mca.get_moving_average_efficiency()) + " " +
                str(self.sqa.get_moving_average()) + " " +
                str(self.ra.get_moving_average()) + "\n"
            )

        if not self.position_file is None:
            for j, quad in enumerate(quads):
                self.position_file.write(
                    "{}, {}, {}, {}, {}, {}\n".format(
                        i, j, quad.x, quad.y, quad.z,
                        quad.get_sensor_radius()
                    )
                )
