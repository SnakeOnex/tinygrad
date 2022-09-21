import numpy as np
import math
import json
import matplotlib.pyplot as plt

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

def generate_skidpad_track():

    def rotate_vec(vec, angle):
        angle = np.deg2rad(angle)
        R = np.array([[np.cos(angle), np.sin(angle)],
                      [-np.sin(angle), np.cos(angle)]])

        print(R.shape)
        print(vec.shape)
        return (R @ vec.T).T

    # params
    track_width = 3.
    inner_diam = 15.25

    car_pos = [0., 0.]
    car_heading = 0.

    # 1. big orange cones
    left_x = -track_width/2
    right_x = track_width/2

    big_cones = [[left_x, 9.5], [right_x, 9.5],
                 [left_x, 10.5], [right_x, 10.5]]

    # 2. inner and outer cirles
    left_center = np.array([left_x - inner_diam/2,10.])
    right_center = np.array([right_x + inner_diam/2,10.])

    yellow_cones = []
    orange_cones = []
    blue_cones = [[left_x - inner_diam/2,10.], [right_x + inner_diam/2,10.]]

    inner_vec = np.array([[0., inner_diam/2]])
    outer_vec = np.array([[0., inner_diam/2 + 3]])

    inner_vec = rotate_vec(inner_vec, 10.)

    for i in range(0, 16):
        angle_step = 360. / 16

        inner_left = left_center + rotate_vec(inner_vec, i*angle_step)
        inner_right = right_center + rotate_vec(inner_vec, i*angle_step)

        outer_left = left_center + rotate_vec(outer_vec, i*angle_step)
        outer_right = right_center + rotate_vec(outer_vec, i*angle_step)

        blue_cones.append([inner_left[:,0], inner_left[:,1]])
        yellow_cones.append([inner_right[:,0], inner_right[:,1]])

        if outer_left[0,0] < left_x:
            yellow_cones.append([outer_left[:,0], outer_left[:,1]])

        if outer_right[0,0] > right_x:
            blue_cones.append([outer_right[:,0], outer_right[:,1]])


    # 3. start and finish zone

    ## start
    start_cones = [[left_x, 1.], [right_x, 1.]]
    [orange_cones.append(s) for s in start_cones]

    for i in range(3):
        orange_cones.append([left_x, (inner_diam+5) + i*3.])
        orange_cones.append([right_x, (inner_diam+5) + i*3.])

    for i in range(4):
        orange_cones.append([left_x + i * (track_width/3), (inner_diam+5) + 3*3.])

    track_dict = {
        "car_position" : car_pos,
        "car_heading"  : car_heading,
        "yellow_cones" : yellow_cones,
        "blue_cones"   : blue_cones,
        "orange_cones" : orange_cones,
        "big_cones"    : big_cones,
    }
    return track_dict

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

def plot_map(map_dict):
    # 1. car pos
    car_pos = np.array(map_dict["car_position"])
    plt.plot(car_pos[0], car_pos[1], '.', color='black')

    # 2. cones
    yellow_cones = np.array(map_dict["yellow_cones"]).reshape(-1, 2)
    blue_cones = np.array(map_dict["blue_cones"]).reshape(-1, 2)
    orange_cones = np.array(map_dict["orange_cones"]).reshape(-1, 2)
    big_cones = np.array(map_dict["big_cones"]).reshape(-1, 2)

    plt.plot(yellow_cones[:,0], yellow_cones[:,1], '.', color='yellow', label="yellow_cones")
    plt.plot(blue_cones[:,0], blue_cones[:,1], '.', color='blue', label="blue_cones")
    plt.plot(orange_cones[:,0], orange_cones[:,1], '.', color='orange', label="orange_cones")
    plt.plot(big_cones[:,0], big_cones[:,1], '.', color='red', label="big_orange_cones")

    plt.axis('equal')
    plt.legend()

    plt.show()


if __name__ == '__main__':
    acc_dict = generate_acceleration_track(3.)
    skidpad_dict = generate_skidpad_track()

    plot_map(skidpad_dict)

    # with open("acceleration_map.json", 'w') as f:
        # json.dump(acc_dict, f, indent=4)








