
import random
import math
import scipy.optimize as opt
import point
import lawnmower
import scipy.stats
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

    def constrain(self, x, y):
        ret_x = x
        ret_y = y

        b_x = False
        b_y = False

        if x < 0:
            ret_x = 0
            b_x = True
        elif x >= self.problem.width:
            ret_x = self.problem.width - 1
            b_x = True

        if y < 0:
            ret_y = 0
            b_y = True
        elif ret_y >= self.problem.height:
            ret_y = self.problem.height - 1
            b_y = True

        return ret_x, ret_y, b_x or b_y

    def get_sample_direction(self, angle, quad, beta):
        beta = math.radians(beta)
        sample_radius_ma = quad.get_ellipse_major() + self.radius_ext
        sample_radius_mi = quad.get_ellipse_minor() + self.radius_ext
        inner_angle = angle - self.angle_range / 2

        time_dict = dict()
        total_time = 0.0
        counter = 0

        while inner_angle < angle + self.angle_range / 2:
            old_x = int(
                quad.get_ellipse_center_dist() +
                sample_radius_ma *
                math.cos(inner_angle)
            )
            old_y = int(
                sample_radius_mi *
                math.sin(inner_angle)
            )

            x = old_x * math.cos(beta) - old_y * math.sin(beta) + quad.x
            y = old_y * math.cos(beta) + old_x * math.sin(beta) + quad.y
            x, y, out = self.constrain(x, y)

            if out:
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

            vec_x_y = point.Point(
                x - e_x, y - e_y
            )

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

        if self.problem.camera_angle_freedom < num_samples:
            num_samples_phi = self.problem.camera_angle_freedom + 1
        else:
            num_samples_phi = num_samples

        if 2 * self.problem.orientation_freedom < num_samples:
            num_samples_beta = 2 * self.problem.orientation_freedom
        else:
            num_samples_beta = num_samples

        if (
                self.problem.initial_camera_angle -
                self.problem.camera_angle_freedom < 0
        ):
            initial_sample_phi = 0
        else:
            initial_sample_phi = self.problem.initial_camera_angle -\
                    self.problem.camera_angle_freedom

        sample_phis = self.get_random_list(
            initial_sample_phi,
            self.problem.initial_camera_angle +
            self.problem.camera_angle_freedom + 1,
            num_samples_phi
        )

        sample_betas = self.get_random_list(
            quad.beta - self.problem.orientation_freedom,
            quad.beta + self.problem.orientation_freedom + 1,
            num_samples_beta
        )

        for n_p in sample_phis:
            quad.set_camera_angle(n_p)
            for n_b in sample_betas:
                new_direction, n_time = self.get_instance_direction(
                    quad, n_b
                )

                if min_time is None or n_time < min_time:
                    min_time = n_time
                    min_direction = new_direction
                    min_beta = n_b
                    min_phi = n_p

        if time.time() - min_time < self.time_threshold:
            return point.Point(0, 0), min_beta, min_phi

        return min_direction, min_beta, min_phi

    def get_next_configuration(self, quad):
        heading, beta, phi  = self.get_new_direction(quad)
        new_z = self.determine_height(quad)
        uheading = heading.to_unit_vector()
        uheading.set_z(new_z - quad.get_z())
        return uheading, beta, phi

    def step(self):
        for quad in self.quad_list:
            self.update_quad(quad)

        return self.quad_list

    def get_random_list(self, nmin, nmax, num):
        ret_list = list()
        for _ in xrange(num):
            ret_list.append(random.randint(nmin, nmax))

        return ret_list


class PlannerMonotonic(PlannerInterface):

    def risk(self, x, y, z):
        init_risk = float(self.risk_grid.get_risk(x, y))
        risk_val = (1 - math.pow(
            float(z - self.problem.min_height) /
            (
                init_risk *
                (self.problem.max_height - self.problem.min_height)
            ), 2
        ))

        return risk_val

    def sq(self, z):
        return math.pow(self.problem.min_height / float(z), 2)

    def get_risk_sq_func(self, x, y):
        def risk_sq(z):
            return self.risk(x, y, z) - self.sq(z)

        return risk_sq

    def determine_height(self, quad):
        quad.intify()
        x, y = quad.x, quad.y
        risk_sq_func = self.get_risk_sq_func(x, y)
        res = opt.fsolve(risk_sq_func, self.problem.max_height)
        return int(max(res))


class PlannerGaussian(PlannerInterface):

    def risk(self, x, y, z):
        init_risk = float(self.risk_grid.get_risk(x, y))
        norm_dist = scipy.stats.norm(
            0, init_risk * self.problem.risk_constant
        )

        return init_risk * norm_dist.pdf(z) / norm_dist.pdf(0)

    def sq(self, z, phi):
        hyp_dist = z / math.cos(math.radians(phi))
        norm_dist = scipy.stats.norm(
            self.problem.sq_height,
            self.problem.sq_std
        )

        return norm_dist.pdf(hyp_dist) / norm_dist.pdf(self.problem.sq_height)

    def get_risk_sq_func(self, x, y, phi):
        def risk_sq(z):
            return self.risk(x, y, z) - self.sq(z, phi)

        return risk_sq

    def determine_height(self, quad):
        quad.intify()
        risk_sq_func = self.get_risk_sq_func(quad.x, quad.y, quad.phi)

        sample_eps = 5
        sample_min = quad.z - sample_eps
        sample_max = quad.z + sample_eps
        sample_range = range(sample_min, sample_max + 1)

        j_list = map(risk_sq_func, sample_range)

        opt_val = min(list(enumerate(j_list)), key=lambda v: v[1])

        return opt_val[0] - sample_eps + quad.z


planners = {
    "rover_monotonic": PlannerMonotonic,
    "rover_gaussian": PlannerGaussian,
    "lawnmower": lawnmower.LawnMower
}
