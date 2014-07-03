
import random
import math
import point
import numpy as np
import copy


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
        self.init_num_samples()

    def init_num_samples(self):
        self.num_surf_samples = 100
        self.num_pos_samples = 10

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

    def get_new_direction(self, quad):
        sample_phis = self.get_camera_angle_samples()
        sample_betas = self.get_orientation_samples(quad)
        position_samples = self.get_position_samples(quad)

        work_quad = copy.deepcopy(quad)

        min_time = None
        min_phi = None
        min_beta = None
        min_pos = None

        for phi in sample_phis:
            for beta in sample_betas:
                for position in position_samples:
                    work_quad.set_x(position.x)
                    work_quad.set_y(position.y)
                    work_quad.set_camera_angle(phi)

                    surface_samples = self.get_surface_samples(work_quad, beta)
                    total_time = 0.0

                    for p in surface_samples:
                        total_time += self.problem.grid[p.x, p.y]

                    if min_time is None or total_time < min_time:
                        min_time = total_time
                        min_phi = phi
                        min_beta = beta
                        min_pos = position

        min_direction = (min_pos - quad).to_unit_vector()

        return min_direction, min_beta, min_phi

    def rotate(self, x, y, x0, y0, angle):
        xn = x * math.cos(angle) - y * math.sin(angle) + x0
        yn = y * math.cos(angle) + x * math.sin(angle) + y0
        return xn, yn

    def get_surface_samples(self, quad, beta):
        ps = list()
        r_ma = int(quad.get_ellipse_major())
        r_mi = int(quad.get_ellipse_minor())
        h = quad.get_ellipse_center_dist()
        k = 0

        xs_o = np.linspace(h - r_ma, h + r_ma, self.num_surf_samples)
        ys_o = np.linspace(k - r_mi, k + r_mi, self.num_surf_samples)

        for x, y in zip(xs_o, ys_o):
            el_eval_x = math.pow(x - h, 2) / math.pow(r_ma, 2)
            el_eval_y = math.pow(y - k, 2) / math.pow(r_mi, 2)

            if el_eval_x + el_eval_y <= 1:
                xr, yr = self.rotate(x, y, quad.x, quad.y, beta)
                if self.inside_workspace(xr, yr):
                    ps.append(point.Point(xr, yr))
        return ps

    def get_position_samples(self, quad):
        angles = np.linspace(0, 2 * math.pi, self.num_pos_samples)
        ps = list()
        for angle in angles:
            r = 10 * self.problem.step_size
            x = r * math.cos(angle) + quad.x
            y = r * math.sin(angle) + quad.y

            if not self.inside_workspace(x, y):
                continue

            ps.append(point.Point(x, y))

        return ps

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
        heading, beta, phi = self.get_new_direction(quad)
        new_z = self.determine_height(quad)
        uheading = heading.to_unit_vector()
        uheading.set_z(new_z - quad.get_z())
        return uheading, beta, phi

    def get_random_list(self, nmin, nmax, num):
        nmin = int(nmin)
        nmax = int(nmax)
        num = int(num)
        ret_list = list()
        for _ in xrange(num):
            ret_list.append(random.randint(nmin, nmax))

        return ret_list


""" For higher order functionality """

import plannergauss
import plannermono
import lawnmower


planners = {
    "rover_monotonic": plannermono.PlannerMonotonic,
    "rover_gaussian": plannergauss.PlannerGaussian,
    "lawnmower": lawnmower.LawnMower
}
