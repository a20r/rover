
import quadcopter
import math
import point


class Directions(object):
    RIGHT = "right"
    LEFT = "left"


class LawnMower(object):

    def __init__(self, problem, *args):
        self.problem = problem
        self.quad_list = self.init_quads()
        self.current_heading = [0, 1]
        self.right = [1, 0]
        self.left = [-1, 0]
        self.up = [0, -1]
        self.down = [0, 1]
        self.current_sweep = Directions.RIGHT

    def init_quads(self):
        quad_list = list()
        init_length = math.ceil(math.sqrt(self.problem.num_quads))
        for i in xrange(self.problem.num_quads):
            down = int(i // init_length)
            accross = int(i % init_length)

            s_x, s_y = (
                25 * self.problem.quad_size * accross,
                25 * self.problem.quad_size * down
            )

            quad_list.append(quadcopter.Quadcopter(
                s_x, s_y, self.problem.min_height,
                self.problem
            ))

        return quad_list

    def update_current_heading(self):
        new_heading = self.current_heading
        for quad in self.quad_list:
            if quad.y >= self.problem.height:

                going_left = self.current_heading == self.left
                going_right = self.current_heading == self.right
                if going_left or going_right:
                    new_heading = self.up
                    break

                if self.current_sweep == Directions.RIGHT:
                    new_heading = self.right
                elif self.current_sweep == Directions.LEFT:
                    new_heading = self.left

            elif quad.y <= 0:

                going_left = self.current_heading == self.left
                going_right = self.current_heading == self.right
                if going_left or going_right:
                    new_heading = self.down
                    break

                if self.current_sweep == Directions.RIGHT:
                    new_heading = self.right
                elif self.current_sweep == Directions.LEFT:
                    new_heading = self.left

            if quad.x <= 0:
                self.current_sweep = Directions.RIGHT
            elif quad.x >= self.problem.width:
                self.current_sweep = Directions.LEFT

        self.current_heading = new_heading

    def step(self):
        self.update_current_heading()
        for quad in self.quad_list:
            quad.move_2d(point.Point(
                self.current_heading[0], self.current_heading[1]
            ))

        return self.quad_list
