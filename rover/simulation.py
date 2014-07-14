
import point
import stats
import plot
import math
import quadcopter
import rospy
import tf
import violations
import controller
import zmqros
from geometry_msgs.msg import Twist


class Simulation(object):

    def __init__(self, problem, risk_grid, **kwargs):
        rospy.init_node("rover", anonymous=True)

        self.init_problem_instance(problem, risk_grid, kwargs)
        self.init_configurations(problem, risk_grid, kwargs)
        self.init_visualizations(problem, risk_grid, kwargs)
        self.init_statistics(problem, risk_grid, kwargs)
        self.init_zmqros(problem)

        self.msg_type = "geometry_msgs/Twist"
        self.max_z_vel = 50  # cm/s

    def init_zmqros(self, problem):
        if self.practical:
            self.swarm = zmqros.server.swarm.create_swarm_from_ns(
                zmqros.get_ns_host(), zmqros.get_ns_port()
            )
        else:
            self.ns = None

    def init_problem_instance(self, problem, risk_grid, kwargs):
        # problem instance setup
        self.problem = problem
        self.risk_grid = risk_grid
        self.planner_obj = kwargs.get("algorithm")
        self.prev_waypoints = dict()
        self.practical = kwargs.get("practical", False)
        self.names = kwargs.get("names", dict())

    def init_configurations(self, problem, risk_grid, kwargs):
        # ROS stuff
        self.listener = tf.TransformListener()
        self.pubs = self.init_pubs(self.names)
        self.quad_list = self.init_quads()
        self.controllers = self.init_controllers()
        self.pl = self.planner_obj(problem, risk_grid, self.quad_list)

    def init_visualizations(self, problem, risk_grid, kwargs):
        # visualization variables
        self.drawer = kwargs.get("drawer", None)
        self.show_time_grid = kwargs.get("show_time_grid", True)

    def init_statistics(self, problem, risk_grid, kwargs):
        # statistics gathering classes
        self.mca = stats.MonteCarloArea(problem, 1000)
        self.sqa = stats.SensorQualityAverage(self.pl)
        self.ra = stats.RiskAverage(self.pl)
        self.atd = stats.AverageTimeDifference(self.problem.grid)
        self.init_stats(**kwargs)

    def init_pubs(self, names):
        pub_dict = dict()
        for name, topic_name in names.iteritems():
            #  YO Change this shit bro
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

    def init_controllers(self):
        Kp, Ki, Kd = 1, 0, 0.5
        controllers = dict()
        for quad in self.quad_list:
            clr = controller.PID(quad, Kp, Ki, Kd)
            controllers[quad] = clr

        return controllers

    def init_quads(self):
        if self.practical:
            quad_list = self.init_quads_practical()
        else:
            quad_list = self.init_quads_simulation()

        for quad in quad_list:
            self.problem.grid.update_grid(quad, 1)
            self.prev_waypoints[quad] = (quad.x, quad.y, quad.z, quad.beta)

        return quad_list

    def saturate(self, val, max_val):
        if val > max_val:
            return max_val
        else:
            return val

    def publish_practical_configuration(self, quad, heading, beta, phi):

        vel = Twist()
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = math.radians(beta - quad.beta)

        vel.linear.x = heading.x * quad.speed
        vel.linear.y = heading.y * quad.speed
        vel.linear.z = self.saturate(50 * heading.z / 5.0, self.max_z_vel)

        pub_vel = self.convert_coordinates_vicon(vel)

        self.swarm[quad.get_name()].send_message(
            self.msg_type,
            self.names[quad.get_name()],
            pub_vel
        )

        expected = [0, 0, 0, 0]
        expected[0] = quad.x + vel.linear.x * quad.speed
        expected[1] = quad.y + vel.linear.y * quad.speed
        expected[2] = quad.z + vel.linear.z
        expected[3] = beta

        return tuple(expected)

    def publish_simulated_configuration(self, quad, heading, beta, phi):
        waypoint = point.Point(
            quad.x + heading.get_x() * quad.speed,
            quad.y + heading.get_y() * quad.speed,
            quad.z + heading.get_z()
        )

        self.controllers[quad].publish_waypoint(waypoint)

        #  Uncomment this line if dont you want dynamics dude
        #  quad.set_position(waypoint.x, waypoint.y, waypoint.z)

        quad.set_orientation(beta)
        quad.set_camera_angle(phi)
        return waypoint.x, waypoint.y, waypoint.z, quad.beta

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

    def publish_hover(self, quad, iteration):
        expected = self.publish_configuration(
            quad, point.Point(0, 0, 0), quad.beta, quad.phi, iteration
        )
        return expected

    def publish_towards_center(self, quad, vio, iteration):
        vc = point.Point(0, 0, 0)
        if vio == violations.X_OUT:
            vc.x = self.problem.width / 2 - quad.x
            uvc = vc.to_unit_vector()
        elif vio == violations.Y_OUT:
            vc.y = self.problem.height / 2 - quad.y
            uvc = vc.to_unit_vector()
        elif vio == violations.Z_OUT:
            vc.z = self.problem.sq_height - quad.z
            uvc = vc

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
            return False, violations.X_OUT, None

        if quad.y > self.problem.height or quad.y < 0:
            return False, violations.Y_OUT, None

        too_high = quad.z > self.problem.max_height
        too_low = quad.z < self.problem.min_height

        if too_high or too_low:
            return False, violations.Z_OUT, None

        for q in self.quad_list:
            distance = q.get_point_2d().dist_to(quad.get_point_2d())
            too_close = distance < self.problem.min_safe_distance
            is_self = q == quad
            if too_close and not is_self:
                return False, violations.TOO_CLOSE, q

        return True, violations.NONE, None

    def run(self):
        if not self.drawer is None:
            self.drawer.draw_risk_grid(self.risk_grid)

        for i in xrange(self.problem.num_steps):
            for quad in self.quad_list:
                try:
                    rpx, rpy, rpz, rb = self.get_configuration(quad)
                except tf.Exception as e:
                    rpx, rpy, rpz, rb = quad.x, quad.y, quad.z, quad.beta
                    print e
                finally:
                    quad.set_position(rpx, rpy, rpz)
                    quad.set_orientation(rb)
                    self.execute_control(quad, i)

                    self.write_verification_results(
                        (rpx, rpy, rpz, rb),
                        self.prev_waypoints[quad], i
                    )

            self.visualize()
            self.update_stats(i)
            self.write_stats_results()

    def execute_control(self, quad, i):
        if self.practical:
            self.problem.grid.update_grid(quad, i + 2)

        conf_allowed, vio, extra = self.is_safe(quad)

        if conf_allowed:
            heading, beta, phi = self.pl\
                .get_next_configuration(quad)

            expected = self.publish_configuration(
                quad, heading, beta, phi, i
            )
        else:
            if vio == violations.TOO_CLOSE:
                heading = (quad.get_position() - extra)\
                    .to_unit_vector()
                expected = self.publish_configuration(
                    quad, heading, quad.beta, quad.phi, i
                )
            else:
                expected = self.publish_towards_center(
                    quad, vio, i
                )

        self.prev_waypoints[quad] = expected

    def visualize(self):
        if self.show_time_grid:
            self.t_plotter.update()

        if not self.drawer is None:
            self.drawer.clear_all()
            #self.drawer.draw_risk_grid(self.risk_grid)

            for quad in self.quad_list:
                self.drawer.draw_quad(quad)

            self.drawer.update()

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
                str(self.atd.get_average()) + " " +
                str(self.pl.mb.get_average_blur()) + "\n"
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
