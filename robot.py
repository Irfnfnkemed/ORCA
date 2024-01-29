import numpy

from calculate import norm, Halfplane


class Robot:
    def __init__(self, radius, x_init, y_init, x_tar, y_tar, v_max):
        self.x_trace_list = None
        self.y_trace_list = None
        self.radius = radius / 0.95  # set radius bigger than the real radius, acting as buffer area
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
