
from planner import PlannerInterface
import math
import scipy.stats


class PlannerGaussian(PlannerInterface):

    def risk(self, x, y, z):
        init_risk = float(self.risk_grid.get_risk(x, y))

        if init_risk == 0:
            return 0

        norm_dist = scipy.stats.norm(
            0, init_risk * self.problem.risk_constant
        )

        return init_risk * norm_dist.pdf(z) / norm_dist.pdf(0)

    def norm_scaled(self, x, mu, sigma):
        return math.exp(-0.5 * math.pow((x - mu) / sigma, 2))

    def sq(self, z, phi):
        hyp_dist = z / math.cos(math.radians(phi))
        return self.norm_scaled(
            hyp_dist, self.problem.sq_height, self.problem.sq_std
        )

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
