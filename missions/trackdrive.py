import numpy as np
import time
import sys
import math
import multiprocessing as mp

from config import path_planner_opt

from algorithms.path_tracking import stanley_steering
from algorithms.path_planning import PathPlanner
from algorithms.general import get_big_orange_distance

from algorithms.speed_profile import SpeedProfile
from algorithms.dima_path_planning import PathPlanner, stanley_smooth_path
from algorithms.dima_optimized_path_planning import PathPlanner as OptimizedPathPlanner
from algorithms.dima_optimized_path_planning import stanley_smooth_path as optimized_smooth_path
from algorithms.david_planning import PathPlanner as DavidPathPlanner
from algorithms.path_smoothing import PathSmoothing


class Trackdrive():
    ID = "Trackdrive"

    def __init__(self):
        mp.Process.__init__(self)

        # CONTROLS CONFIGURATION
        self.lookahead_dist = 1.7
        self.linear_gain = 1.7
        self.nonlinear_gain = 1.2

        # speed profile constants
        max_safe_speed = 5.75  # in m/s

        self.david_path_planner = DavidPathPlanner()
        self.path_planner = PathPlanner()
        self.speed_profile = SpeedProfile()
        self.optimized_path_planner = OptimizedPathPlanner()
        self.path_smoothing = PathSmoothing()

        self.use_speed_profile = True
        self.speed_set_point = 3.  # m/s
        self.torque_set_point = 5.

        # mission planning variables
        self.finished = False
        self.start_timestamp = None

        self.last_lap_time = 0.
        self.finish_time = float('inf')
        self.stopped_time = None

        self.laps_to_drive = 1
        self.laps_driven = 0

    def loop(self, **kwargs):
        """
        args:
          percep_data - Nx3 np.array containing cone predictions
        ret:
          finished
          steering angle set point
          speed setpoint
          debug_dict
          path
        """

        percep_data = kwargs["percep_data"]
        wheel_speed = kwargs["wheel_speed"]

        # 0. save starting time stamp during the first loop
        if self.start_timestamp is None:
            self.start_timestamp = time.perf_counter()
        time_since_start = time.perf_counter() - self.start_timestamp
        time_since_last_lap = time_since_start - self.last_lap_time

        # path = self.old_path_planner.find_path(percep_data)
        path = self.david_path_planner.find_path(percep_data)
        path, _ = self.path_smoothing.torch_smooth(path)

        # Speed profile
        if self.use_speed_profile and len(path) > 2 and self.speed_set_point > 0.:
            speed_arr = self.speed_profile.michals_profile(path, wheel_speed)
            self.speed_set_point = speed_arr[1]

        # 2. planning
        # if have been driving for more than 3 seconds since passing start/finish, start looking for it again
        if time_since_last_lap > 3.:
            dist_to_finish = get_big_orange_distance(
                cone_preds=percep_data, min_big_cones=1)

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
        delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)
        torque = self.torque_set_point
        debug_dict = {
            "time_since_start": time_since_start,
            "lap_time": time_since_last_lap,
            "finish_time": self.finish_time,
            "laps_driven": f"{self.laps_driven} / {self.laps_to_drive}",
            "finished": self.finished,
        }

        return self.finished, delta, self.speed_set_point, torque, debug_dict, path, controller_log["target"]
