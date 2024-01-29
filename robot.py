import numpy

from calculate import norm, Halfplane


class Robot:
    def __init__(self, radius, x_init, y_init, x_tar, y_tar, v_max):
        self.x_trace_list = None
        self.y_trace_list = None
        self.radius = radius / 0.95  # set radius bigger than the real radius, acting as buffer area
        self.pos_init = numpy.array([x_init, y_init])
        self.pos_cur = numpy.array([x_init, y_init])
        self.pos_tar = numpy.array([x_tar, y_tar])
        self.v_max = v_max
        self.v_cur = self.pos_tar - self.pos_cur
        self.v_cur = self.v_cur / norm(self.v_cur) * v_max
        self.halfplane_list = []
        self.trace = []
        self.danger = 0

    def clear_halfplane(self):
        self.halfplane_list.clear()

    def set_halfplane(self, normal_vec, point):
        self.halfplane_list.append(Halfplane(normal_vec, point))

    def set_trace(self):
        self.trace.append(numpy.array(self.pos_cur))

    def update_pos(self):
        self.pos_cur += self.v_cur * 0.1

    def set_trace_component(self):
        self.x_trace_list = [pos[0] for pos in self.trace]
        self.y_trace_list = [pos[1] for pos in self.trace]


#
# robot_a = Robot(1.5, 0.0, 0.0, 22.0, 21.0, 2.7)
# robot_b = Robot(1.5, 3.0, 3.0, 19.0, 18.0, 3.0)
# robot_c = Robot(1.5, 6.0, 6.0, 16.0, 15.0, 2.5)
# robot_d = Robot(1.5, 9.0, 9.0, 13.0, 11.5, 3.2)
# robot_e = Robot(1.5, 12.0, 12.0, 10.0, 9.0, 3.3)
# robot_f = Robot(1.5, 15.0, 15.0, 7.0, 6.0, 3.5)
# robot_g = Robot(1.5, 18.0, 18.0, 4.0, 3.0, 2.6)
# robot_h = Robot(1.5, 21.0, 21.0, 1.0, 0.0, 3.6)
# robot_i = Robot(1.5, 0.0, 20.0, 20.0, 0.0, 3.6)
# robot_j = Robot(1.5, 20.0, 0.0, 0.0, 20.0, 3.6)
# robot_k = Robot(1.5, 5.0, 15.0, 15.0, 5.0, 3.6)
# robot_l = Robot(1.5, 15.0, 5.0, 5.0, 15.0, 3.6)
# robot_list = [robot_a, robot_b, robot_c, robot_d, robot_e, robot_f, robot_g, robot_h, robot_i, robot_j, robot_k, robot_l]
#
# robot_a = Robot(3.0, 0.0, 0.0, 10.0, 10.0, 3.6)
# robot_b = Robot(4.0, 7.0, 0.0, 0.0, 0.0, 3.4)
# robot_c = Robot(4.0, 0.0, 7.0, 0.0, 10.0, 3.3)
# robot_list = [robot_a, robot_b, robot_c]




