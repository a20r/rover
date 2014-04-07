
import timegrid
import pygame.color as color
import drawer


class Colors(object):

    def __init__(self, **kwargs):
        self.color_dict = color.THECOLORS
        self.quad = self.color_dict[kwargs.get("quad", "blue")]
        self.quad_sensor = self.color_dict[kwargs.get("quad_sensor", "green")]
        self.background = self.color_dict[kwargs.get("background", "white")]


class Problem(object):

    def __init__(self, **kwargs):
        self.width = kwargs.get("width", 640)
        self.height = kwargs.get("height", 480)
        self.num_quads = kwargs.get("num_quads", 10)
        self.min_height = kwargs.get("min_height", 60)
        self.max_height = kwargs.get("max_height", 100)
        self.step_size = kwargs.get("step_size", 10)
        self.quad_size = kwargs.get("quad_size", 4)
        self.viewing_angle = kwargs.get("viewing_angle", 30)
        self.grid = timegrid.TimeGrid(self.width, self.height)
        self.num_steps = kwargs.get("num_steps", 250)
        self.delay = kwargs.get("delay", 0.1) # seconds
