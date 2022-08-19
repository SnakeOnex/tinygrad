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

