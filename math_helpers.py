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

def global_to_local(cones, car_pos, car_heading):
    """
    """

    # car_heading *= -1.
    car_heading = np.deg2rad(car_heading)
    R = np.array([[np.cos(car_heading), np.sin(car_heading)],
                  [-np.sin(car_heading), np.cos(car_heading)]])

    cones[:,0:2] -= car_pos

    # print("R: ", R.shape)
    # print("cones: ", cones.shape)
    cones = (R @ cones.T).T

    return cones

def rotate_around_point(angle,position,point):
    rot_x = np.cos(np.deg2rad(angle)-np.pi/2)*point[0] -np.sin(np.deg2rad(angle)-np.pi/2)*point[1]
    rot_y = np.sin(np.deg2rad(angle)-np.pi/2)*point[0] - np.cos(np.deg2rad(angle)-np.pi/2)*point[1]
    return np.array((rot_x, rot_y))
