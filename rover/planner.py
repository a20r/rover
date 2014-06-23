
import random
import math
import point
import lawnmower
import time


class PlannerInterface(object):

    def __init__(self, problem, risk_grid, quads):
        self.problem = problem
        self.quad_spacing = 20
        self.initial_orientation = 0
        self.risk_grid = risk_grid
        self.num_samples = 100
        self.radius_ext = 8
        self.angle_range = math.pi / 8
        self.angle_step = 2 * math.pi / self.num_samples
        self.quad_list = quads
        self.time_threshold = 1

    def inside_workspace(self, x, y):
        b_x = True
        b_y = True

        if x < 0:
            b_x = False
        elif x >= self.problem.width:
            b_x = False

        if y < 0:
            b_y = False
        elif y >= self.problem.height:
            b_y = False

        return b_x and b_y

    def get_sample_direction(self, angle, quad, beta):
        beta = math.radians(beta)
        sample_radius_ma = quad.get_ellipse_major() + self.radius_ext
        sample_radius_mi = quad.get_ellipse_minor() + self.radius_ext
        inner_angle = angle - self.angle_range / 2

        time_dict = dict()
        total_time = 0.0
        counter = 0

        while inner_angle < angle + self.angle_range / 2:
            x_off = quad.get_ellipse_center_dist()
            old_x = int(x_off + sample_radius_ma * math.cos(inner_angle))
            old_y = int(sample_radius_mi * math.sin(inner_angle))

            x = old_x * math.cos(beta) - old_y * math.sin(beta) + quad.x
            y = old_y * math.cos(beta) + old_x * math.sin(beta) + quad.y

            if not self.inside_workspace(x, y):
                raise ValueError()

            time_dict[(x, y, inner_angle)] = self.problem.grid[x, y]
            total_time += self.problem.grid[x, y]
            counter += 1

            inner_angle += self.angle_step

        avg_x = 0.0
        avg_y = 0.0
        avg_angle = 0.0
        for (x, y, i_angle), t in time_dict.iteritems():
            weight = t / total_time
            avg_x += x * weight
            avg_y += y * weight
            avg_angle += i_angle * weight

        return avg_x, avg_y, avg_angle, total_time / counter

    def get_instance_direction(self, quad, beta):
        """
        Returns the unit vector of the direction the quad should go
        """

        angle = float(0)
        min_time = None
        min_x_y = None

        while angle < 2 * math.pi + self.angle_range:
            try:
                x, y, i_angle, avg_time = self.get_sample_direction(
                    angle, quad, beta
                )
            except ValueError:
                continue
            finally:
                angle += self.angle_range

            e_x, e_y = quad.get_ellipse_center()
            vec_x_y = point.Point(x - e_x, y - e_y)

            if min_time is None or min_time > avg_time:
                min_time = avg_time
                min_x_y = vec_x_y

        return min_x_y, min_time

    def get_new_direction(self, quad):
        min_time = None
        min_direction = None
        min_beta = None
        min_phi = None
        num_samples = 10

        sample_phis = self.get_camera_angle_samples()
        sample_betas = self.get_orientation_samples(quad)

        for n_p in sample_phis:
            quad.set_camera_angle(n_p)
            for n_b in sample_betas:
                new_direction, n_time = self.get_instance_direction(quad, n_b)

                if min_time is None or n_time < min_time:
                    min_time = n_time
                    min_direction = new_direction
                    min_beta = n_b
                    min_phi = n_p

        return min_direction, min_beta, min_phi

    def get_camera_angle_samples(self):
        num_samples = 10
        phi_sub_phi_free = self.problem.initial_camera_angle\
                - self.problem.camera_angle_freedom

        if self.problem.camera_angle_freedom < num_samples:
            num_samples_phi = self.problem.camera_angle_freedom + 1
        else:
            num_samples_phi = num_samples

        if phi_sub_phi_free < 0:
            initial_sample_phi = 0
        else:
            initial_sample_phi = phi_sub_phi_free

        sample_phis = self.get_random_list(
            initial_sample_phi,
            self.problem.initial_camera_angle +
            self.problem.camera_angle_freedom + 1,
            num_samples_phi
        )

        return sample_phis

    def get_orientation_samples(self, quad):
        num_samples = 10
        if 2 * self.problem.orientation_freedom < num_samples:
            num_samples_beta = 2 * self.problem.orientation_freedom
        else:
            num_samples_beta = num_samples

        sample_betas = self.get_random_list(
            quad.beta - self.problem.orientation_freedom,
            quad.beta + self.problem.orientation_freedom + 1,
            num_samples_beta
        )

        return sample_betas

    def get_next_configuration(self, quad):
        heading, beta, phi  = self.get_new_direction(quad)
        new_z = self.determine_height(quad)
        uheading = heading.to_unit_vector()
        uheading.set_z(new_z - quad.get_z())
        return uheading, beta, phi

    def get_random_list(self, nmin, nmax, num):
        ret_list = list()
        for _ in xrange(num):
            ret_list.append(random.randint(nmin, nmax))

        return ret_list


""" For higher order functionality """

import plannergauss
import plannermono


planners = {
    "rover_monotonic": plannermono.PlannerMonotonic,
    "rover_gaussian": plannergauss.PlannerGaussian,
    "lawnmower": lawnmower.LawnMower
}
