import random

from animation import Animation
from calculate import *
from robot import Robot


class Control:
    def __init__(self, ax_range=50, time_interval=0.1, length=1000, step=0.2, rate=0.5):
        self.ax_range = ax_range
        self.time_interval = time_interval
        self.length = length
        self.step = step
        self.rate = rate
        self.robot_list = []

    def add_robot(self, radius, x_init, y_init, x_tar, y_tar, v_max, time_interval):
        self.robot_list.append(Robot(radius, x_init, y_init, x_tar, y_tar, v_max, time_interval))

    def check_radius(self, radius, pos_x, pos_y):
        for robot in self.robot_list:
            if (robot.pos_cur[0] - pos_x) ** 2 + (robot.pos_cur[1] - pos_y) ** 2 < (radius + robot.radius * 0.95) ** 2:
                return False
        return True

    def find_min_change_velocity(self, robot_cur, robot_abj):
        velocity = robot_cur.v_cur - robot_abj.v_cur  # relative velocity
        pos = (robot_abj.pos_cur - robot_cur.pos_cur) / self.time_interval  # centre of the circle in velocity coordinate system
        radius = (robot_cur.radius + robot_abj.radius) / self.time_interval  # radius of the circle in velocity coordinate system
        sin_include = radius / norm(pos)
        cos_include = math.sqrt(1 - min(1.0, sin_include ** 2))
        rotation_matrix_cw = numpy.array([[cos_include, sin_include], [-sin_include, cos_include]])  # observing from the positive z-direction
        rotated_matrix_ccw = numpy.array([[cos_include, -sin_include], [sin_include, cos_include]])
        tangent_vec_cw = numpy.dot(rotation_matrix_cw, pos)
        tangent_vec_cw /= norm(tangent_vec_cw)
        tangent_vec_ccw = numpy.dot(rotated_matrix_ccw, pos)
        tangent_vec_ccw /= norm(tangent_vec_ccw)
        if dot(velocity, tangent_vec_cw) < norm(pos) * cos_include and dot(velocity, tangent_vec_ccw) < norm(pos) * cos_include:
            if numpy.sum((velocity - pos) ** 2) < radius ** 2:
                return (velocity - pos) * (-1 + radius / norm(velocity - pos)), velocity - pos
            else:
                return (velocity - pos) * (-1 + radius / norm(velocity - pos)), velocity - pos
        else:
            if cross(pos, velocity) > 0:  # counterclockwise side
                return numpy.array([-tangent_vec_ccw[1], tangent_vec_ccw[0]]) * cross(velocity, tangent_vec_ccw), numpy.array([-tangent_vec_ccw[1], tangent_vec_ccw[0]])
            else:  # clockwise side
                return -numpy.array([tangent_vec_cw[1], -tangent_vec_cw[0]]) * cross(velocity, tangent_vec_cw), numpy.array([tangent_vec_cw[1], -tangent_vec_cw[0]])

    def linear_program(self, robot):
        target_pos = robot.pos_tar - robot.pos_cur
        if norm(robot.v_emergency) > robot.v_max / 20:
            target = 0.4 * robot.v_emergency * robot.v_max + 0.6 * robot.v_cur
        else:
            length = norm(target_pos)
            if length < robot.v_max * self.time_interval:
                target = target_pos / self.time_interval
            else:
                target = target_pos / length * robot.v_max
            if (robot.danger >= 3 and numpy.sum(robot.v_cur ** 2) < (0.1 * robot.v_max) ** 2 and robot.bigger < robot.danger * 0.8) or robot.lock >= 20:
                target = numpy.array([-target[1], target[0]])  # possible deadlock, attempting to leave the deadlock area
            else:
                target = robot.v_cur * 0.8 + target * 0.2  # near multiple objects, conservative selection method
        target += robot.v_max * numpy.array([(random.random() - 0.5) / 40, (random.random() - 0.5) / 40])
        optimal = target
        for i in range(len(robot.halfplane_list)):
            if dot(optimal, robot.halfplane_list[i].normal_vec) >= robot.halfplane_list[i].constant:
                continue  # no new restrictions introduced
            else:
                foot_drop = target + (robot.halfplane_list[i].constant - dot(target, robot.halfplane_list[i].normal_vec)) * robot.halfplane_list[i].normal_vec
                foot_drop_origin = robot.halfplane_list[i].constant * robot.halfplane_list[i].normal_vec
                if numpy.sum(foot_drop_origin ** 2) > robot.v_max ** 2:
                    return self.linear_program_2(robot.halfplane_list, robot.v_max, robot.danger)  # no solution, yet has feasible area
                line_vec_cw = numpy.array([robot.halfplane_list[i].normal_vec[1], -robot.halfplane_list[i].normal_vec[0]]) * math.sqrt(robot.v_max ** 2 - numpy.sum(foot_drop_origin ** 2))
                line_vec_ccw = -line_vec_cw
                point_cw = foot_drop_origin + line_vec_ccw
                point_ccw = foot_drop_origin + line_vec_cw
            for j in range(i):
                tmp = find_cross_point(robot.halfplane_list[i], robot.halfplane_list[j])
                if cross(robot.halfplane_list[i].normal_vec, robot.halfplane_list[j].normal_vec) < 0:  # clockwise
                    if tmp is not None and cross(robot.halfplane_list[i].normal_vec, tmp - point_cw) < 0:
                        point_cw = tmp
                else:
                    if tmp is not None and cross(robot.halfplane_list[i].normal_vec, tmp - point_ccw) > 0:
                        point_ccw = tmp
            if cross(robot.halfplane_list[i].normal_vec, point_ccw - point_cw) > 0:  # no solution
                return self.linear_program_2(robot.halfplane_list, robot.v_max, robot.danger)
            elif cross(robot.halfplane_list[i].normal_vec, foot_drop - point_ccw) < 0:
                optimal = point_ccw
            elif cross(robot.halfplane_list[i].normal_vec, foot_drop - point_cw) > 0:
                optimal = point_cw
            else:
                optimal = foot_drop
        return optimal

    def linear_program_2(self, halfplane_list, v_max, danger):
        optimal = numpy.array([0.0, 0.0])
        top_dis, decent_vec = find_top_plane(halfplane_list, optimal)
        stop = False
        while not stop:
            stop = True
            tmp_step = self.step
            while norm(optimal + decent_vec * tmp_step) > v_max:
                tmp_step *= self.rate
            while tmp_step > self.step * (self.rate ** 5):
                tmp_dis, tmp_vec = find_top_plane(halfplane_list, optimal + decent_vec * tmp_step)
                if tmp_dis < top_dis:
                    optimal += decent_vec * tmp_step
                    top_dis = tmp_dis
                    decent_vec = tmp_vec
                    stop = False
                    break
                tmp_step *= self.rate
        return optimal

    def find_new_velocity(self):
        for robot in self.robot_list:
            robot.clear_halfplane()
            robot.danger = 0
            robot.v_emergency = numpy.array([0.0, 0.0])
            if numpy.sum(robot.v_cur ** 2) < (0.2 * robot.v_max) ** 2:
                robot.lock += 1
            else:
                robot.lock = 0
        for i in range(len(self.robot_list)):
            for j in range(i):
                dis_square = numpy.sum((self.robot_list[i].pos_cur - self.robot_list[j].pos_cur) ** 2)
                min_square = (self.robot_list[i].radius + self.robot_list[j].radius) ** 2
                if dis_square < 1.7 * min_square:
                    self.robot_list[i].danger += 1
                    self.robot_list[j].danger += 1
                    if self.robot_list[i].radius > 2 * self.robot_list[j].radius:
                        self.robot_list[i].bigger += 1
                    if self.robot_list[j].radius > 2 * self.robot_list[i].radius:
                        self.robot_list[j].bigger += 1
                if dis_square <= min_square and \
                        numpy.sum((self.robot_list[i].pos_cur - self.robot_list[i].pos_tar) ** 2) > (0.1 * self.robot_list[i].radius) ** 2 and \
                        numpy.sum((self.robot_list[j].pos_cur - self.robot_list[j].pos_tar) ** 2) > (0.1 * self.robot_list[j].radius) ** 2:
                    self.robot_list[i].v_emergency = normalization(normalization(self.robot_list[i].pos_cur - self.robot_list[j].pos_cur) + self.robot_list[i].v_emergency)
                    self.robot_list[j].v_emergency = normalization(normalization(self.robot_list[j].pos_cur - self.robot_list[i].pos_cur) + self.robot_list[j].v_emergency)
                if dis_square < (self.robot_list[i].v_max * self.time_interval + self.robot_list[j].v_max * self.time_interval + self.robot_list[i].radius + self.robot_list[j].radius) ** 2:
                    random_float = (random.random() - 0.5) / 5
                    u, vec = self.find_min_change_velocity(self.robot_list[i], self.robot_list[j])  # only need to consider the robot which is possible to collide
                    self.robot_list[i].set_halfplane(vec, self.robot_list[i].v_cur + (self.robot_list[j].radius / (self.robot_list[i].radius + self.robot_list[j].radius) + random_float) * u)
                    self.robot_list[j].set_halfplane(-vec, self.robot_list[j].v_cur - (self.robot_list[i].radius / (self.robot_list[i].radius + self.robot_list[j].radius) - random_float) * u)
        for robot in self.robot_list:
            if len(robot.halfplane_list) == 0:
                if numpy.sum((robot.pos_tar - robot.pos_cur) ** 2) > (robot.v_max * self.time_interval) ** 2:
                    robot.v_cur = 0.2 * robot.v_max * normalization(robot.pos_tar - robot.pos_cur) + 0.8 * robot.v_cur
                else:
                    robot.v_cur = (robot.pos_tar - robot.pos_cur) / self.time_interval
            else:
                robot.v_cur = self.linear_program(robot)

    def run(self):
        for i in range(self.length * 5):
            if i % 5 == 0:
                for robot in self.robot_list:
                    robot.set_trace()
            self.find_new_velocity()
            for robot in self.robot_list:
                robot.update_pos()
        for robot in self.robot_list:
            robot.set_trace_component()

    def show(self):
        self.run()
        animation = Animation(self.robot_list, self.length, self.ax_range)
        animation.show()
