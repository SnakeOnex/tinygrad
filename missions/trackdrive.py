import numpy as np
import time
import sys
import math
import multiprocessing as mp
from multiprocessing import shared_memory

from config import path_planner_opt
from nodes.can1_node import Can1RecvItems, Can1SendItems

from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner
from algorithms.general import get_big_orange_distance

class Trackdrive():
    ID = "Trackdrive"

    def __init__(self, perception_out, can1_recv_state):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_state = can1_recv_state

        # CONTROLS CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        self.path_planner = PathPlanner(path_planner_opt)

        self.speed_set_point = 3.

        ## mission planning variables
        self.finished = False
        self.start_timestamp = None

        self.last_lap_time = 0.
        self.finish_time = float('inf')
        self.stopped_time = None

        self.laps_to_drive = 100
        self.laps_driven = 0

    def loop(self, cone_preds):
        """
        args:
          cone_preds - Nx3 np.array containing cone predictions
        ret:
          finished
          steering angle set point
          speed setpoint
          debug_dict
          path
        """
        
        # 0. save starting time stamp during the first loop
        if self.start_timestamp is None:
            self.start_timestamp = time.perf_counter()
        time_since_start = time.perf_counter() - self.start_timestamp
        time_since_last_lap = time_since_start - self.last_lap_time

        # 1. receive perception data
        wheel_speed = self.can1_recv_state[Can1RecvItems.wheel_speed.value]
        path = self.path_planner.find_path(cone_preds)

        # 2. planning
        ## if have been driving for more than 3 seconds since passing start/finish, start looking for it again
        if time_since_last_lap > 3.:
            dist_to_finish = get_big_orange_distance(cone_preds=cone_preds, min_big_cones=1)

            if dist_to_finish is not None:
                finish_in = dist_to_finish / wheel_speed
                self.finish_time = time_since_start + finish_in

        if self.finish_time < time_since_start and (self.laps_driven + 1) < self.laps_to_drive:
            self.laps_driven += 1
            self.last_lap_time = time_since_start
            self.finish_time = float('inf')
        elif self.finish_time < time_since_start and (self.laps_driven + 1) == self.laps_to_drive:
            self.speed_set_point = 0.

            # if car stopped, set self.finished to True
            if self.stopped_time is None and wheel_speed <= 0.1:
                self.laps_driven += 1
                self.stopped_time = time_since_start

        if self.stopped_time is not None and self.stopped_time + 1. < time_since_start:
            self.finished = True

        # 2. controls
        delta, _, log = stanley_steering(path, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {
            "time_since_start": time_since_start,
            "lap_time": time_since_last_lap,
            "finish_time": self.finish_time,
            "laps_driven": self.laps_driven,
            "speed_setpoint": self.speed_set_point,
            "stopped_time": self.stopped_time,
            "finished": self.finished
        }

        return self.finished, delta, self.speed_set_point, debug_dict, path
