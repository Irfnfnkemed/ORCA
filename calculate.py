import math

import numpy


class Halfplane:
    def __init__(self, normal_vec, point):
        self.normal_vec = normal_vec / norm(normal_vec)  # normal_vec is unit normal vector pointing to feasible half space
        self.constant = dot(point, self.normal_vec)  # the condition in the feasible half space is <x,n> >= constant


def find_cross_point(halfplane_lhs, halfplane_rhs):
    denominator = cross(halfplane_lhs.normal_vec, halfplane_rhs.normal_vec)
    if denominator == 0:
        return None
    else:
        numerator_x = halfplane_lhs.constant * halfplane_rhs.normal_vec[1] - halfplane_rhs.constant * halfplane_lhs.normal_vec[1]
        numerator_y = halfplane_lhs.normal_vec[0] * halfplane_rhs.constant - halfplane_rhs.normal_vec[0] * halfplane_lhs.constant
        return numpy.array([numerator_x, numerator_y]) / denominator


def find_top_plane(halfplane_list, pos):
    top_dis = None
    decent_vec = None
    for halfplane in halfplane_list:
        dis = halfplane.constant - dot(halfplane.normal_vec, pos)
        if top_dis is None or dis > top_dis:
            top_dis = dis
            decent_vec = halfplane.normal_vec
    return top_dis, decent_vec / norm(decent_vec)


def cross(vec_lhs, vec_rhs):  # two-dimensional, the third coordinate is 0
    return vec_lhs[0] * vec_rhs[1] - vec_lhs[1] * vec_rhs[0]


def dot(vec_lhs, vec_rhs):  # two-dimensional dot
    return vec_lhs[0] * vec_rhs[0] + vec_lhs[1] * vec_rhs[1]


def norm(vec):  # two-dimensional, L2 norm
    return math.sqrt(vec[0] ** 2 + vec[1] ** 2)
