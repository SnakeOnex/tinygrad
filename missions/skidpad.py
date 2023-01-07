import multiprocessing as mp
# ! temporary
import zmq
import pickle
import numpy as np
import math
import sys

from config import path_planner_opt
from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner

CONE_CLASSES = {"yellow": 0.,
                "blue": 1.,
                "orange": 2.,
                "big": 3.}

# ! temporary array for sim testing
SIM_WAYPOINTS = {'start_left': np.array([-3.18173506,  3.12047806]),
                 'start_right': np.array([3.32004047, 3.02806534]),
                 'end_left': np.array([-3.32004047, 16.97193466]),
                 'end_right': np.array([3.18173506, 16.87952194]),
                 'center': np.array([0., 8.])}


class Skidpad():
    ID = "Skidpad"
    WAYPOINT_RADIUS_SQ = 1.5

    def __init__(self):
        mp.Process.__init__(self)

        # CONTROLLER CONFIGURATION
        self.lookahead_dist = 1.8
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        self.path_planner = PathPlanner(path_planner_opt)
        self.finish_detect = False

        self.speed_set_point = 15.
        self.finished = False

        # SKIPAD WAYPOINT CONFIGURATION
        self.waypoint_state = False  #  is false when waypoints haven't been initialized
        self.waypoints = {key: [SIM_WAYPOINTS[key], False, 0]
                          for key in SIM_WAYPOINTS.keys()}
        self.cone_boolean_mask = "all"
        self.keep_straight = True
        self.waypoints["center"][1] = True
        # ! temporary
        context = zmq.Context()
        self.glob_coord_socket = context.socket(zmq.SUB)
        self.glob_coord_socket.connect("tcp://127.0.0.1:50004")
        self.glob_coord_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.glob_coord_poller = zmq.Poller()
        self.glob_coord_poller.register(self.glob_coord_socket, zmq.POLLIN)

    def loop(self, world_state, wheel_speed):
        """
        args:
          vision output (path, cone positions)
        ret:
          steering_angle
          wheelspeed_setpoint
        """
        # 0. get global coordinates
        while self.glob_coord_poller.poll(0.):
            data = self.glob_coord_socket.recv()
            self.update_waypoint_state(pickle.loads(data))
        # 1. receive perception data
        path = self.path_planner.find_path(self.filter_state(world_state))
        # consider moving wheel speed to mission node

        delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {}


        if self.keep_straight:
            delta = 0.

        return self.finished, delta, self.speed_set_point, debug_dict, (path, controller_log["target"])

    def update_waypoint_state(self, coords):
        current_passed_zone = None
        for key, value in self.waypoints.items():
            if value[1]:
                if np.sum(np.square(coords-value[0])) <= Skidpad.WAYPOINT_RADIUS_SQ:
                    current_passed_zone = key
                    break

        if current_passed_zone != None:
            n_passes = self.waypoints[current_passed_zone][2]
            print("N_passes: ", n_passes)
            print("Curr zone: ", current_passed_zone)
            new_cone_mask = None
            next_active_zone = None
            if current_passed_zone == "center":
                if n_passes == 0:
                    self.keep_straight = False

                if n_passes == 0 or n_passes == 1:
                    new_cone_mask = "blue_only"
                    next_active_zone = "end_left"
                elif n_passes == 2 or n_passes == 3:
                    new_cone_mask = "yellow_only"
                    next_active_zone = "end_right"
                else:
                    new_cone_mask = "orange_only"
                    self.finish_detect = True
            elif current_passed_zone == "start_left":
                new_cone_mask = "no_blue"
                next_active_zone = "center"
            elif current_passed_zone == "start_right":
                new_cone_mask = "no_yellow"
                next_active_zone = "center"
            elif current_passed_zone == "end_left":
                new_cone_mask = "all"
                next_active_zone = "start_left"
            elif current_passed_zone == "end_right":
                new_cone_mask = "all"
                next_active_zone = "start_right"

            self.waypoints[current_passed_zone][2] += 1
            self.waypoints[current_passed_zone][1] = False
            self.waypoints[next_active_zone][1] = True
            self.cone_boolean_mask = new_cone_mask

    def filter_state(self, word_state):
        if self.cone_boolean_mask == "all":
            return word_state
        elif self.cone_boolean_mask == "blue_only":
            return word_state[(word_state[:, 2] == CONE_CLASSES["blue"])]
        elif self.cone_boolean_mask == "no_blue":
            return word_state[(word_state[:, 0] >= 0) & (word_state[:, 2] != CONE_CLASSES["blue"])]
        elif self.cone_boolean_mask == "no_yellow":
            return word_state[(word_state[:, 0] <= 0)]
        elif self.cone_boolean_mask == "yellow_only":
            return word_state[(word_state[:, 2] == CONE_CLASSES["yellow"]) & (word_state[:, 0] >= 0)]
        elif self.cone_boolean_mask == "orange_only":
            return word_state[word_state[:, 2] == CONE_CLASSES["orange"]]
