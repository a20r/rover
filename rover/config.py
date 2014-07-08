
import timegrid


class Problem(object):

    def __init__(self, **kwargs):
        self.width = kwargs.get("width", 1000)
        self.height = kwargs.get("height", 600)
        self.num_quads = kwargs.get("num_quads", 40)
        self.min_height = kwargs.get("min_height", 60)
        self.max_height = kwargs.get("max_height", 200)
        self.step_size = kwargs.get("step_size", 10)
        self.quad_size = kwargs.get("quad_size", 4)
        self.viewing_angle = kwargs.get("viewing_angle", 30)
        self.num_steps = kwargs.get("num_steps", 1000)
        self.sq_height = kwargs.get("sq_height", 130)
        self.sq_std = kwargs.get("sq_std", 20)
        self.ns_host = kwargs.get("ns_host", "localhost")
        self.ns_port = kwargs.get("ns_port", 8000)
        self.risk_constant = kwargs.get("risk_constant", 100)
        self.orientation_freedom = kwargs.get("orientation_freedom", 10)
        self.camera_angle_freedom = kwargs.get("camera_angle_freedom", 10)
        self.initial_camera_angle = kwargs.get("initial_camera_angle", 15)
        self.min_safe_distance = kwargs.get("min_safe_distance", 100)
        self.grid = timegrid.TimeGrid(self.width, self.height, self)
