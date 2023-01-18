import numpy as np
from ursina import *


def angle_to_vector(angle):
    vec = np.array([0., 1.])
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

    car_heading = np.deg2rad(car_heading)
    R = np.array([[np.cos(car_heading), np.sin(car_heading)],
                  [-np.sin(car_heading), np.cos(car_heading)]])

    cones[:, 0:2] -= car_pos
    cones = (R @ cones.T).T

    return cones


def local_to_global(cones, car_pos, car_heading):
    car_heading = np.deg2rad(car_heading)

    R = np.array([[np.cos(car_heading), np.sin(car_heading)],
                  [-np.sin(car_heading), np.cos(car_heading)]])
    R = np.linalg.inv(R)

    cones = (R @ cones.T).T
    cones[:, 0:2] += car_pos
    return cones


def rotate_around_point(angle, position, point):
    rot_x = np.cos(np.deg2rad(angle)-np.pi/2)*point[0] - np.sin(np.deg2rad(angle)-np.pi/2)*point[1]
    rot_y = np.sin(np.deg2rad(angle)-np.pi/2)*point[0] - np.cos(np.deg2rad(angle)-np.pi/2)*point[1]
    return np.array((rot_x, rot_y))


def filter_occluded_cones(cones_local, occlusion_profile):
    """
    Filters out cones outside of the occlusion profile
    args:
      cones_local - Nx3 np.array
      occlusion_profile - 4 element tuple of floats (x_min, x_max, y_min, y_max)
    ret:
      cones_filtered - Mx3 np.array
    """
    left_max, right_max, forward_min, forward_max = occlusion_profile

    mask_lr = (cones_local[:, 0] <= right_max) & (cones_local[:, 0] >= left_max)
    mask_f = (cones_local[:, 1] <= forward_max) & (cones_local[:, 1] >= forward_min)
    mask = mask_lr & mask_f

    cones_filtered = cones_local[mask, :]
    return cones_filtered
