
import planner
import scipy.optimize as opt
import math


class PlannerMonotonic(planner.PlannerInterface):

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
