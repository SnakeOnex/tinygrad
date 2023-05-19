import numpy as np
import time
import sys
import math
import multiprocessing as mp

from config import path_planner_opt

from algorithms.path_tracking import stanley_steering
from algorithms.old_path_planning import OldPathPlanner
from algorithms.general import get_big_orange_distance

from algorithms.speed_profile import SpeedProfile
from algorithms.path_planning import PathPlanner, stanley_smooth_path
from algorithms.optimized_path_planning import PathPlanner as OptimizedPathPlanner
from algorithms.optimized_path_planning import stanley_smooth_path as optimized_smooth_path
from algorithms.david_planning import PathPlanner as DavidPathPlanner
from algorithms.path_planning_more_points import PathPlanner as PathPlannerMorePoints
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
        max_safe_speed = 5.75 # in m/s

        self.old_path_planner = OldPathPlanner(path_planner_opt)
        self.david_path_planner = DavidPathPlanner()
        self.path_planner = PathPlanner()
        self.speed_profile = SpeedProfile(max_safe_speed)
        self.optimized_path_planner = OptimizedPathPlanner()
        self.more_points_path_planner = PathPlannerMorePoints()
        self.path_smoothing = PathSmoothing()

        self.use_speed_profile = True
        self.use_new_path_planning = False
        self.speed_set_point = 6.

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

        # 1. receive perception data
        if self.use_new_path_planning:
            # start_time = time.perf_counter()
            # path = self.path_planner.find_path(percep_data)
            path = self.optimized_path_planner.find_path(percep_data)
            # path = stanley_smooth_path(path)
            path, _ = self.path_smoothing.torch_smooth(path)

            # path = optimized_smooth_path(path) # minimize func

            # add more path points (+3?)
            # path = optimized_smooth_path(path, use_spline_as_smoother=False, add_more_points_to_path=True) # a liitle bit slower, but same problems as with spline...
            # path = optimized_smooth_path(path, use_spline_as_smoother=True) # only spline

            # print("new took: ", time.perf_counter() - start_time)
            # start_time = time.perf_counter()
            # path = self.old_path_planner.find_path(percep_data)
            # print("old took: ", time.perf_counter() - start_time)
        else:
            # path = self.old_path_planner.find_path(percep_data)
            # path = stanley_smooth_path(path)
            # path = self.more_points_path_planner.find_path(percep_data)
            path = self.david_path_planner.find_path(percep_data)
            path, _ = self.path_smoothing.torch_smooth(path)
            # path, _ = self.path_smoothing.scipy_smooth(path)

        # Speed profile
        if self.use_speed_profile and len(path) > 2 and self.speed_set_point > 0.:
            speed_arr = self.speed_profile.compute_speed_profile(path, wheel_speed)
            self.speed_set_point = speed_arr[1]
        # print(f"{speed_arr.shape=}")
        # print(f"{path.shape=}")

        # print("Set speed from speed profile:", self.speed_set_point)
        # print("Wheel speed 1:", wheel_speed)

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
        delta, controller_log = stanley_steering(
            path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {
            "time_since_start": time_since_start,
            "lap_time": time_since_last_lap,
            "finish_time": self.finish_time,
            "laps_driven": f"{self.laps_driven} / {self.laps_to_drive}",
            "finished": self.finished,
        }

        return self.finished, delta, self.speed_set_point, debug_dict, path, controller_log["target"]
