import numpy as np
from ursina import *

def angle_to_vector(angle):
    vec = np.array([0.,1.])
    angle = np.deg2rad(angle)

    R = np.array([[np.cos(angle), np.sin(-angle)],
                 [np.sin(angle), np.cos(angle)]])

    vec = R @ vec
    vec /= np.linalg.norm(vec)

    # return np.array((vec[0], 0., vec[1]))
    return vec

def vec_to_3d(vec, y=0.):
    return Vec3(vec[0], y, vec[1])

def num2str(num, pad_to=7):
    num_str = f"{num:.2f}"
    return ((pad_to - len(num_str)) * " ") + num_str

def global_to_local(cones, car_heading, car_position):
    """
    """
    prepath = np.matmul(np.array([[np.cos(-self.alpha),np.sin(-self.alpha)],[-np.sin(-self.alpha),np.cos(-self.alpha)]]),prepath.T).T

    R = np.array([[np.cons(-self.car_heading)]])

    cones[:,0:2] -= car_position

