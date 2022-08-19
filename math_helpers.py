import numpy as np

def angle_to_vector(self, angle):
    vec = np.array([0.,1.])
    angle = np.deg2rad(angle)

    R = np.array([[np.cos(angle), np.sin(-angle)],
                 [np.sin(angle), np.cos(angle)]])

    vec = R @ vec
    vec /= np.linalg.norm(vec)

    return np.array((vec[0], 0., vec[1]))
