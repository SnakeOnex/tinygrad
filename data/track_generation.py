import numpy as np
import math
import json

def generate_skidpad_path(alpha, x_t, y_t):
        R = np.array([[np.cos(alpha), -np.sin(alpha), x_t],
                      [np.sin(alpha), np.cos(alpha), y_t],
                      [0, 0, 1]])
        track = []
        r_skid = 9.125
        n = 50
        for i in range(10):
            o = R.dot(np.array([0,
                                (10 / 10) * i, 1]))
            track.append((o[0], o[1]))
        for k in range(2):
            for i in range(n):
                o = R.dot(np.array([r_skid + math.cos(2 * math.pi / n * (n / 2 - i)) * r_skid,
                                    10.625 + math.sin(2 * math.pi / n * (n / 2 - i)) * r_skid, 1]))
                track.append((o[0], o[1]))
        for k in range(2):
            for i in range(n):
                o = R.dot(np.array([-r_skid + math.cos(2 * math.pi / n * (i + 1)) * r_skid,
                                    10.625 + math.sin(2 * math.pi / n * (i + 1)) * r_skid, 1]))
                track.append((o[0], o[1]))
        for i in range(10):
            o = R.dot(np.array([0, 10 + (11 / 10) * i, 1]))
            track.append([o[0], o[1]])
        return track

def generate_acceleration_track(width):

    left_x = -width/2
    right_x = width/2

    # 1. start position
    car_pos = [0., 0.]
    car_heading = 0.

    big_cones = [[left_x, 0.], [right_x, 0.], [left_x, 1.], [right_x, 1.],
                 [left_x, 74.], [right_x, 74.], [left_x, 75.], [right_x, 75.]]

    blue_cones = []
    yellow_cones = []

    for i in range(1, 74, 5):
        blue_cones.append([left_x, float(i)])
        yellow_cones.append([right_x, float(i)])

    orange_cones = []

    for i in range(80, 180, 5):
        orange_cones.append([left_x, float(i)])
        orange_cones.append([right_x, float(i)])

    track_dict = {
        "car_position" : car_pos,
        "car_heading"  : car_heading,
        "yellow_cones" : yellow_cones,
        "blue_cones"   : blue_cones,
        "orange_cones" : orange_cones,
        "big_cones"    : big_cones,
    }
    return track_dict

if __name__ == '__main__':
    acc_dict = generate_acceleration_track(3.)

    with open("acceleration_map.json", 'w') as f:
        json.dump(acc_dict, f, indent=4)

