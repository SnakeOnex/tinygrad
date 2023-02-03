import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import json
from enum import IntEnum

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
        # self.car_heading = out["car_heading"]
        self.car_heading = 0.

        print(self.car_heading)

        # Control variables
        self.mode = Modes.ADD_CONE # add_cone, delete_cone, car_pos, car_dir
        self.add_cone_color = Cones.YELLOW # 0 = yc, 1 = bc, 2 = oc, 3 = boc

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

        self.fig.canvas.mpl_connect("button_press_event", self.click_handler)
        self.fig.canvas.mpl_connect("key_press_event", self.key_press_handler)

    def click_handler(self, event):
        if self.mode == Modes.ADD_CONE:
            new_cone = np.array([event.xdata, event.ydata, self.add_cone_color])
            self.all_cones = np.vstack((self.all_cones, new_cone))
        elif self.mode == Modes.DELETE_CONE:
            click_pos = np.array([event.xdata, event.ydata]).reshape((1,2))
            dists = norm(self.all_cones[:,0:2] - click_pos, axis=1)
            mask = np.where(dists > min(dists))[0]
            self.all_cones = self.all_cones[mask,:]
        elif self.mode == Modes.CHANGE_CAR_POS:
            pass #TODO
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
        elif event.key == 'h':
            print("\n".join([
                "h - help",
                "m - change mode(ADD_CONE, DEL_CONE, CHANGE_CAR_POS, CHANGE_CAR_DIR)",
                "a - change to ADD_CONE mode",
                "d - change to DEL_CONE mode",
                "c - change add_cone_color"])
            )

    def redraw(self):

        # draw car vector
        angle = np.deg2rad(self.car_heading)
        R = np.array([[np.cos(angle), np.sin(angle)],
                      [-np.sin(angle), np.cos(angle)]])

        ## TODO
        unit_vector = np.array([[0,1]]).reshape((2,1))
        dir_vec = (R @ unit_vector).T
        self.ax.quiver([self.car_pos[0,0]], [self.car_pos[0,1]], [dir_vec[0,0]], [dir_vec[0,1]], angles='xy', scale_units='x', scale=1)

        # draw lines
        self.drawn_start.set_data(self.start_line[:,0], self.start_line[:,1])
        self.drawn_finish.set_data(self.finish_line[:,0], self.finish_line[:,1])

        # draw cones
        yc, bc, oc, boc = self.get_cones_by_color()
        self.drawn_yellows.set_data(yc[:,0], yc[:,1])
        self.drawn_blues.set_data(bc[:,0], bc[:,1])
        self.drawn_orange.set_data(oc[:,0], oc[:,1])
        self.drawn_big.set_data(boc[:,0], boc[:,1])

        min_x, min_y = np.min(self.all_cones[:,0:2] ,axis=0)
        max_x, max_y = np.max(self.all_cones[:,0:2] ,axis=0)

        self.ax.set_xlim(min(-30, min_x-10), max(max_x+10, 30))
        self.ax.set_ylim(min(-30, min_y-10), max(max_y+10, 30))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def get_cones_by_color(self):
        yc = self.all_cones[self.all_cones[:,2] == 0, :]
        bc = self.all_cones[self.all_cones[:,2] == 1, :]
        oc = self.all_cones[self.all_cones[:,2] == 2, :]
        boc = self.all_cones[self.all_cones[:,2] == 3, :]
        return yc, bc, oc, boc

if __name__ == "__main__":
    parser = argparse.ArgumentParser() 
    parser.add_argument('--map', type=str, default="new_map.json")
    args = parser.parse_args()

    map_editor = MapEditor(map_path=Path(args.map))
    map_editor.redraw()

    plt.show()