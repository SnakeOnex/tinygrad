import numpy as np
import json
import matplotlib.pyplot as plt
from pathlib import Path

def inner_outer_to_json(inner_path, outer_path, json_path):
    yc = np.load(outer_path)
    bc = np.load(inner_path)

    out = {
        "car_position" : [0., 0.],
        "car_heading" : 0.,
        "yellow_cones" :  yc.tolist(),
        "blue_cones" :  bc.tolist(),
        "orange_cones" : [],
        "big_cones" : []
    }

    with open(json_path, 'w') as f:
        json.dump(out, f, indent=4)

def generate_slam_autox_track():
    yc = np.load(outer_path)
    bc = np.load(inner_path)

    out = {
        "car_position" : [0., 0.],
        "car_heading" : 0.,
        "yellow_cones" :  yc.tolist(),
        "blue_cones" :  bc.tolist(),
        "orange_cones" : [],
        "big_cones" : []
    }

    with open(json_path, 'w') as f:
        json.dump(out, f, indent=4)

    


if __name__ == '__main__':

    # 0. params
    inner_path = Path("slam_inner.npy")
    outer_path = Path("slam_outer.npy")
    json_path = Path("slam_map.json")

    inner_outer_to_json(inner_path, outer_path, json_path)

    # 1. load cone positions from map file
    with open(json_path, 'r') as f:
        out = json.load(f)

    print(out)
    yc = np.array(out["yellow_cones"])
    bc = np.array(out["blue_cones"])
    oc = np.array(out["big_cones"])

    # 2. visualize the cones
    plt.plot(yc[:,0], yc[:,1], '.', color="yellow")
    plt.plot(bc[:,0], bc[:,1], '.', color="blue")
    plt.axis('equal')
    # plt.plot(oc[:,0], oc[:,1], '.', color="orange")
    plt.show()

    # 3. save into json


