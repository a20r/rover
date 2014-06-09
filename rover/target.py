
import point
import scipy.stats


class Target(object):

    def __init__(self, problem, **kwargs):
        self.problem = problem
        self.pos = kwargs.get("init_pos", point.get_random_point(
            self.problem.width,
            self.problem.height
        ))
        self.goal_radius = kwargs.get("goal_radius", 10)
        self.step_size = kwargs.get("step_size", 10)
        self.std = kwargs.get("std", 100)
        self.current_goal = None
        self.heading = None
        self.scaling_constant = 0.00001

    def get_new_goal(self):
        return point.get_random_point(
            self.problem.width,
            self.problem.height
        )

    def in_goal(self):
        return self.pos.dist_to(self.current_goal) < self.goal_radius

    def step(self):
        if self.heading is None or self.in_goal():
            self.current_goal = self.get_new_goal()
            self.heading = (self.current_goal - self.pos).to_unit_vector()

        self.pos.x += self.heading.x * self.step_size
        self.pos.y += self.heading.y * self.step_size
        return self

    def get_weight(self, x, y):
        p = point.Point(x, y)
        w_raw = p.dist_to(self.pos) ** 2
        return self.scaling_constant * w_raw


