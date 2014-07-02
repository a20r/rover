
import point


class PID(object):

    def __init__(self, quad, Kp, Ki, Kd):
        self.quad = quad
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.noise_std = 10
        self.init_dyn_vars()

    def init_dyn_vars(self):
        self.hx = 0
        self.hy = 0
        self.hz = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.sx = 0
        self.sy = 0
        self.sz = 0

    def publish_waypoint(self, waypoint):
        noise = point.get_noisy_point(self.noise_std)
        dp = waypoint - self.quad
        dp -= noise
        self.update_sigmas(dp)

        self.ax = self.Kp * dp.x - self.Kd * self.hx + self.Ki * self.sx
        self.ay = self.Kp * dp.y - self.Kd * self.hy + self.Ki * self.sy
        self.az = self.Kp * dp.z - self.Kd * self.hz

        self.update_velocities()
        self.update_position()
        return self.quad

    def update_sigmas(self, dp):
        self.sx += dp.x
        self.sy += dp.y
        self.sz += dp.z

    def update_velocities(self):
        # dt = 1
        self.hx += self.ax
        self.hy += self.ay
        self.hz += self.az
        return self

    def update_position(self):
        self.quad.x += self.hx
        self.quad.y += self.hy
        self.quad.z += self.hz
        return self

    def get_Kp(self):
        return self.Kp

    def get_Kd(self):
        return self.Kd

    def set_Kp(self, Kp):
        self.Kp = Kp
        return self

    def set_Kd(self, Kd):
        self.Kd = Kd
        return self
