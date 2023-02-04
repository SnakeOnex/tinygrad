import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import json
from enum import IntEnum

import sys
sys.path.append('..')

from sim.math_helpers import angle_to_vector, global_to_local, filter_occluded_cones
from algorithms.path_planning import PathPlanner

class Cones(IntEnum):
    YELLOW=0,
    BLUE=1,
    ORANGE=2,
    BIG=3

class Modes(IntEnum):
    ADD_CONE = 0,
    DELETE_CONE = 1,
    CHANGE_CAR_POS = 2,
    CHANGE_CAR_DIR = 3

class MapEditor():
    def __init__(self, map_path):
        # load map if exists
        with open(map_path, 'r') as f:
            out = json.load(f)

        self.start_line = np.array(out["start_line"]).reshape(-1,2)
        self.finish_line = np.array(out["finish_line"]).reshape(-1,2)

        yc = np.hstack((out["yellow_cones"], np.full((len(out["yellow_cones"]), 1), 0)))
        bc = np.hstack((out["blue_cones"], np.full((len(out["blue_cones"]), 1), 1)))
        oc = np.hstack((out["orange_cones"], np.full((len(out["orange_cones"]), 1), 2)))
        boc = np.hstack((out["big_cones"], np.full((len(out["big_cones"]), 1), 3)))
        self.all_cones = np.vstack((yc, bc, oc, boc))

        self.car_pos = np.array(out["car_position"]).reshape((1,2))
        self.car_heading = out["car_heading"]
        self.occlusion_profile = [-6., 6., 2.5, 15.]
        self.path_planner = PathPlanner({"n_steps":5})

        print(self.car_heading)

        # Control variables
        self.mode = Modes.ADD_CONE # add_cone, delete_cone, car_pos, car_dir
        self.add_cone_color = Cones.YELLOW # 0 = yc, 1 = bc, 2 = oc, 3 = boc
        self.last_op_msg = ""

        # Matplotlib elements
        self.fig = plt.figure(figsize=(15, 15))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Click Track")

        self.drawn_blues, = self.ax.plot([], [], "o", **{"color": "blue",   "mec": "black"})
        self.drawn_yellows, = self.ax.plot([], [], "o", **{"color": "yellow", "mec": "black"})
        self.drawn_orange, = self.ax.plot([], [], "o", **{"color": "orange", "mec": "black"})
        self.drawn_big, = self.ax.plot([], [], "o", **{"color": "red", "mec": "black"})

        self.drawn_start, = self.ax.plot([], [], "-", **{"color": "red"})
        self.drawn_finish, = self.ax.plot([], [], "-", **{"color": "red"})

        self.drawn_car, = self.ax.plot([self.car_pos[0,0]], [self.car_pos[0,1]], "o", **{"color": "black", "mec":"orange"})
        dir_vec = angle_to_vector(self.car_heading)
        self.drawn_dir, = self.ax.plot([self.car_pos[0,0], self.car_pos[0,0]+dir_vec[0]], [self.car_pos[0,1], self.car_pos[0,1]+dir_vec[1]], "-", **{"color": "black"})

        self.drawn_path, = self.ax.plot([], [], "-", **{"color": "red"})

        self.fig.canvas.mpl_connect("button_press_event", self.click_handler)
        self.fig.canvas.mpl_connect("key_press_event", self.key_press_handler)

    def click_handler(self, event):
        if self.mode == Modes.ADD_CONE:
            new_cone = np.array([event.xdata, event.ydata, self.add_cone_color])
            self.all_cones = np.vstack((self.all_cones, new_cone))
            self.last_op_msg = f"Added {Cones(self.add_cone_color).name} cone to pos ({event.xdata:.2f},{event.ydata:.2f})"
        elif self.mode == Modes.DELETE_CONE:
            click_pos = np.array([event.xdata, event.ydata]).reshape((1,2))
            dists = norm(self.all_cones[:,0:2] - click_pos, axis=1)
            mask = np.where(dists > min(dists))[0]
            self.all_cones = self.all_cones[mask,:]
            self.last_op_msg = f"Deleted a cone"
        elif self.mode == Modes.CHANGE_CAR_POS:
            self.car_pos = np.array([[event.xdata, event.ydata]]).reshape((1,2))
            self.last_op_msg = f"change car pos to ({event.xdata},{event.ydata})"
        elif self.mode == Modes.CHANGE_CAR_DIR:
            pass #TODO

        self.redraw()

    def key_press_handler(self, event):
        if event.key == 'm':
            self.mode = (self.mode + 1) % len(Modes)
            print("change to mode: ", Modes(self.mode))
        elif event.key == 'a':
            self.mode = Modes.ADD_CONE
            print("changed Mode to: ADD_CONE")
        elif event.key == 'd':
            self.mode = Modes.DELETE_CONE
            print("changed Mode to: DELETE_CONE")
        elif event.key == 'c':
            self.add_cone_color = (self.add_cone_color + 1) % len(Cones)
            print("change ADD_CONE_COLOR to: ", Cones(self.add_cone_color))
        elif event.key == 'u':
            self.last_op_msg = f"Saved map into file"
            self.save_to_file()
        elif event.key == 'h':
            print("\n".join([
                "h - help",
                "m - change mode(ADD_CONE, DEL_CONE, CHANGE_CAR_POS, CHANGE_CAR_DIR)",
                "a - change to ADD_CONE mode",
                "d - change to DEL_CONE mode",
                "c - change add_cone_color",
                "u - save to file"])
            )
        self.redraw()

    def redraw(self):

        # draw car vector
        self.drawn_car.set_data([self.car_pos[0,0]], [self.car_pos[0,1]])
        dir_vec = angle_to_vector(self.car_heading)
        self.drawn_dir.set_data([self.car_pos[0,0], self.car_pos[0,0]+dir_vec[0]], [self.car_pos[0,1], self.car_pos[0,1]+dir_vec[1]])

        # draw lines
        self.drawn_start.set_data(self.start_line[:,0], self.start_line[:,1])
        self.drawn_finish.set_data(self.finish_line[:,0], self.finish_line[:,1])

        # draw cones
        yc, bc, oc, boc = self.get_cones_by_color()
        self.drawn_yellows.set_data(yc[:,0], yc[:,1])
        self.drawn_blues.set_data(bc[:,0], bc[:,1])
        self.drawn_orange.set_data(oc[:,0], oc[:,1])
        self.drawn_big.set_data(boc[:,0], boc[:,1])

        # draw path
        cones_local = global_to_local(np.array(self.all_cones[:, 0:2]), self.car_pos, self.car_heading)
        cones_local = np.hstack((cones_local, self.all_cones[:, 2:3]))
        cones_local = filter_occluded_cones(cones_local, self.occlusion_profile)
        cones_local[:, 0] *= -1
        path = self.path_planner.find_path(cones_local) 
        path[:,0] *= -1
        path = path + self.car_pos
        self.drawn_path.set_data(path[:,0], path[:,1])

        min_x, min_y = np.min(self.all_cones[:,0:2] ,axis=0)
        max_x, max_y = np.max(self.all_cones[:,0:2] ,axis=0)

        self.ax.set_xlim(min(-30, min_x-10), max(max_x+10, 30))
        self.ax.set_ylim(min(-30, min_y-10), max(max_y+10, 30))

        self.ax.set_title(f"Mode: {Modes(self.mode).name}, Color: {Cones(self.add_cone_color).name}, Last op: {self.last_op_msg}")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def get_cones_by_color(self):
        yc = self.all_cones[self.all_cones[:,2] == 0, 0:2]
        bc = self.all_cones[self.all_cones[:,2] == 1, 0:2]
        oc = self.all_cones[self.all_cones[:,2] == 2, 0:2]
        boc = self.all_cones[self.all_cones[:,2] == 3, 0:2]
        return yc, bc, oc, boc

    def save_to_file(self):
        yc, bc, oc, boc = self.get_cones_by_color()
        map_dict = {
            "car_position" : self.car_pos.tolist(),
            "car_heading"  : self.car_heading,
            "yellow_cones" : yc.tolist(),
            "blue_cones"   : bc.tolist(),
            "orange_cones" : oc.tolist(),
            "big_cones"    : boc.tolist(),
            "start_line"   : self.start_line.tolist(),
            "finish_line"  : self.finish_line.tolist()
        }

        with open("test_map.json", 'w') as f:
            json.dump(map_dict, f, indent=4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser() 
    parser.add_argument('--map', type=str, default="new_map.json")
    args = parser.parse_args()

    map_editor = MapEditor(map_path=Path(args.map))
    map_editor.redraw()

    plt.show()
