
import pygame
import pygame.color as color

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


    def draw_quad(self, quad):
        pygame.draw.circle(
            self.screen, self.colors.quad,
            quad.get_pos_2d(), self.problem.quad_size
        )

        pygame.draw.circle(
            self.screen, self.colors.quad_sensor,
            quad.get_pos_2d(), int(quad.get_sensor_radius())
        )

        return self


    def update(self):
        pygame.display.flip()
        for e in pygame.event.get():
            if e.type is pygame.QUIT:
                exit()

        return self


    def clear_all(self):
        self.screen.fill(self.colors.background)
        return self
