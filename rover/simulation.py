
import planner
import point
import time
import stats
import plot
import random
import math
import quadcopter
import rospy
import tf
import violations
import plannergauss
from geometry_msgs.msg import Twist


class Simulation(object):

    def __init__(self, problem, risk_grid, **kwargs):
        rospy.init_node("rover", anonymous=True)

        # problem instance setup
        self.problem = problem
        self.risk_grid = risk_grid
        self.planner_obj = kwargs.get("algorithm", plannergauss.PlannerGaussian)
        self.prev_waypoints = dict()
        self.practical = kwargs.get("practical", False)
        names = kwargs.get("names", dict())

        # ROS stuff
        self.listener = tf.TransformListener()
        self.pubs = self.init_pubs(names)
        self.quad_list = self.init_quads()
        self.pl = self.planner_obj(problem, risk_grid, self.quad_list)

        # visualization variables
        self.drawer = kwargs.get("drawer", None)
        self.show_time_grid = kwargs.get("show_time_grid", True)

        # statistics gathering classes
        self.mca = stats.MonteCarloArea(problem, 1000)
        self.sqa = stats.SensorQualityAverage(self.pl)
        self.ra = stats.RiskAverage(self.pl)
        self.atd = stats.AverageTimeDifference(self.problem.grid)
        self.init_stats(**kwargs)

    def init_pubs(self, names):
        pub_dict = dict()
        for name, topic_name in names.iteritems():
            pub_dict[name] = rospy.Publisher(
                topic_name, Twist, queue_size=10
            )
        return pub_dict

    def init_stats(self, **kwargs):
        out_file = kwargs.get("out_file", None)
        position_file = kwargs.get("position_file", None)
        verification_file = kwargs.get("verification_file", None)

        if not out_file is None:
            self.out_file = open(out_file, "w")
        else:
            self.out_file = None

        if not position_file is None:
            self.position_file = open(position_file, "w")
        else:
            self.position_file = None

        if not verification_file is None:
            self.verification_file = open(verification_file, "w")
        else:
            self.verification_file = None

        if self.show_time_grid:
            self.t_plotter = plot.TimeGridPlotter(self.problem.grid)

    def init_quads_practical(self):
        quad_list = list()
        for name, _ in self.pubs.iteritems():
            quad = quadcopter.Quadcopter(self.problem, name)
            x, y, z, b = self.get_configuration(quad)
            quad.set_position(x, y, z)
            quad.set_orientation(math.degrees(b))
            quad_list.append(quad)

        return quad_list

    def init_quads_simulation(self):
        quad_list = list()
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

    def init_quads(self):
        current_time = time.time()
        if self.practical:
            quad_list = self.init_quads_practical()
        else:
            quad_list = self.init_quads_simulation()

        for quad in quad_list:
            self.problem.grid.update_grid(quad, current_time)
            self.prev_waypoints[quad] = (quad.x, quad.y, quad.z, quad.beta)

        return quad_list

    def publish_practical_configuration(self, quad, heading, beta, phi):
        waypoint = Twist()
        waypoint.angular.x = 0
        waypoint.angular.y = 0
        waypoint.angular.z = math.radians(beta)

        waypoint.linear.x = quad.x + heading.x * self.problem.step_size
        waypoint.linear.y = quad.y + heading.y * self.problem.step_size
        waypoint.linear.z = quad.z + heading.z
        pub_waypoint = self.convert_coordinates_vicon(waypoint)

        self.pubs[quad.get_name()].publish(pub_waypoint)
        return waypoint.linear.x, waypoint.linear.y, waypoint.linear.z, beta

    def publish_simulated_configuration(self, quad, heading, beta, phi):
        quad.set_position(
            quad.x + heading.get_x() * self.problem.step_size,
            quad.y + heading.get_y() * self.problem.step_size,
            quad.z + heading.get_z()
        )
        quad.set_orientation(beta)
        quad.set_camera_angle(phi)
        return quad.x, quad.y, quad.z, quad.beta

    def publish_configuration(self, quad, heading, beta, phi, iteration):
        if self.practical:
            expected = self.publish_practical_configuration(
                quad, heading, beta, phi
            )
        else:
            expected = self.publish_simulated_configuration(
                quad, heading, beta, phi
            )
            self.problem.grid.update_grid(quad, iteration + 2)
        return expected

    def publish_hover(self, quad):
        expected = self.publish_configuration(
            quad, point.Point(0, 0, 0), quad.beta, quad.phi
        )
        return expected

    def publish_towards_center(self, quad, vio, iteration):
        vc = point.Point(0, 0, 0)
        if vio == violations.X_OUT:
            vc.x = self.problem.width / 2 - quad.x
        elif vio == violations.Y_OUT:
            vc.y = self.problem.height / 2 - quad.y
        elif vio == violations.Z_OUT:
            vc.z = self.problem.sq_height - quad.z

        uvc = vc.to_unit_vector()
        expected = self.publish_configuration(
            quad, uvc, quad.beta, quad.phi, iteration
        )
        return expected

    def get_configuration(self, quad):
        if self.practical:
            tf_world = "/world"
            our_obj = "/" + quad.get_name() + "/base_link"

            self.listener.waitForTransform(
                tf_world, our_obj,
                rospy.Time(0), rospy.Duration(0.1)
            )

            trans, rot = self.listener.lookupTransform(
                tf_world, our_obj, rospy.Time(0)
            )

            actual_conf = self.convert_coordinates_rviz(trans, rot)

            return actual_conf
        else:
            return quad.x, quad.y, quad.z, quad.beta

    def convert_coordinates_vicon(self, waypoint):
        pos = waypoint.linear
        pos.x = (pos.x - self.problem.width / 2) / 100.0
        pos.y = (pos.y - self.problem.height / 2) / 100.0
        pos.z = pos.z / 100.0
        waypoint.linear = pos
        return waypoint

    def convert_coordinates_rviz(self, trans, rot):
        r_x = trans[0] * 100 + self.problem.width / 2
        r_y = trans[1] * 100 + self.problem.height / 2
        r_z = trans[2] * 100
        _, _, beta = tf.transformations.euler_from_quaternion(rot)
        return r_x, r_y, r_z, math.degrees(beta)

    def is_safe(self, quad):
        if quad.x > self.problem.width or quad.x < 0:
            return False, violations.X_OUT

        if quad.y > self.problem.height or quad.y < 0:
            return False, violations.Y_OUT

        if quad.z > self.problem.max_height or quad.z < self.problem.min_height:
            return False, violations.Z_OUT

        return True, violations.NONE

    def run(self):
        for i in xrange(self.problem.num_steps):
            current_time = time.time()
            for quad in self.quad_list:
                try:
                    rpx, rpy, rpz, rb = self.get_configuration(quad)
                except tf.Exception as e:
                    rpx = quad.x
                    rpy = quad.y
                    rpz = quad.z
                    rb = quad.b
                    print e
                finally:
                    quad.set_position(rpx, rpy, rpz)
                    quad.set_orientation(rb)

                    if self.practical:
                        # if it is not practical, it updates the grid with
                        # the new position in the publish_configuration method
                        self.problem.grid.update_grid(quad, i + 2)

                    self.write_verification_results(
                        (rpx, rpy, rpz, rb),
                        self.prev_waypoints[quad], i
                    )

                    conf_allowed, vio = self.is_safe(quad)

                    if conf_allowed:
                        heading, beta, phi = self.pl.get_next_configuration(
                            quad
                        )

                        expected = self.publish_configuration(
                            quad, heading, beta, phi, i
                        )
                    else:
                        expected = self.publish_towards_center(quad, vio, i)

                    self.prev_waypoints[quad] = expected

            self.visualize()
            self.update_stats(i)
            self.write_stats_results()

    def visualize(self):
        if self.show_time_grid:
            self.t_plotter.update()

        if not self.drawer is None:
            self.drawer.clear_all()
            self.drawer.draw_risk_grid(self.risk_grid)

            for quad in self.quad_list:
                self.drawer.draw_quad(quad)

            frame = self.drawer.update()

    def update_stats(self, iteration):
        self.mca.update_average_efficiency(self.quad_list)
        self.sqa.update_average_sq(self.quad_list)
        self.ra.update_average_risk(self.quad_list)
        self.atd.update_average_time_difference(iteration + 2)

    def write_stats_results(self):
        if not self.out_file is None:
            self.out_file.write(
                str(self.mca.get_moving_average_efficiency()) + " " +
                str(self.sqa.get_moving_average()) + " " +
                str(self.ra.get_moving_average()) + " " +
                str(self.atd.get_average()) + "\n"
            )

    def write_verification_results(self, expected_pos, out_pos, iteration):
        if not self.verification_file is None:
            self.verification_file.write(
                "{} {} {} {} {} {} {} {} {}\n".format(
                    iteration,
                    expected_pos[0],
                    expected_pos[1],
                    expected_pos[2],
                    expected_pos[3],
                    out_pos[0],
                    out_pos[1],
                    out_pos[2],
                    out_pos[3]
                )
            )
