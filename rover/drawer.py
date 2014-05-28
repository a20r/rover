
import pygame
import pygame.color as color
import collections
from visualization_msgs.msg import Marker
import rospy


class Drawer(object):

    def __init__(self, problem, colors):
        pygame.init()

        self.problem = problem
        self.colors = colors
        self.color_dict = color.THECOLORS
        self.screen = pygame.display.set_mode(
            (self.problem.width, self.problem.height)
        )

        self.clear_all().update()
        self.constants = pygame

        self.coverage = collections.deque(list(), 100)
        self.evader_list = collections.deque(list(), 100)

    def draw_evader(self, e_x, e_y):
        self.evader_list.append((e_x, e_y))
        pygame.draw.circle(
            self.screen, self.colors.evader,
            (e_x, e_y), self.problem.evader_size
        )

    def draw_coverage(self):
        for qp, r in self.coverage:
            pygame.draw.circle(
                self.screen, self.colors.coverage,
                qp, r
            )

        return self

    def add_coverage(self, quad):
        self.coverage.append((
            quad.get_pos_2d(), int(quad.get_sensor_radius())
        ))

    def draw_quad(self, quad):
        pygame.draw.circle(
            self.screen, self.colors.quad,
            quad.get_pos_2d(), self.problem.quad_size
        )

        pygame.draw.circle(
            self.screen, self.colors.quad_sensor,
            quad.get_pos_2d(), int(quad.get_sensor_radius()), 4
        )

        return self

    def draw_risk_grid(self, risk_grid):
        for _, _, r_point in risk_grid.get_risk_points():
            pygame.draw.circle(
                self.screen, self.colors.risk_point,
                r_point.to_list(), 10
            )

        return self

    def update(self):
        pygame.display.flip()
        frame = pygame.Surface((
            self.problem.width, self.problem.height
        ))
        frame.blit(self.screen, (0, 0))

        for e in pygame.event.get():
            if e.type is pygame.QUIT:
                exit()

        return frame

    def update_with_frame(self, frame):
        self.screen.blit(frame, (0, 0))
        pygame.display.flip()
        for e in pygame.event.get():
            if e.type is pygame.QUIT:
                exit()

    def get_key_pressed(self):
        return pygame.key.get_pressed()

    def clear_all(self):
        self.screen.fill(self.colors.background)
        return self

    def can_play(self):
        return True
