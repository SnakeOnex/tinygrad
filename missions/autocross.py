import numpy as np
import time
import sys
import math
import multiprocessing as mp

from config import path_planner_opt

from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner
from algorithms.general import get_big_orange_distance

class Autocross():
    ID = "Autocross"

    def __init__(self):
        mp.Process.__init__(self)

        # CONTROLS CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        self.path_planner = PathPlanner(path_planner_opt)

        self.speed_set_point = 10.

        ## mission planning variables
        self.finished = False
        self.start_timestamp = None

        self.brake_time = float('inf')
        self.stopped_time = None

    def loop(self, cone_preds, wheel_speed):
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

        # 1. receive perception data
        path = self.path_planner.find_path(cone_preds)

        # 2. planning
        ## if have been driving for more than 3 seconds, start looking for finish line
        if time_since_start > 3.:
            dist_to_finish = get_big_orange_distance(cone_preds=cone_preds, min_big_cones=1)

            if dist_to_finish is not None:
                brake_in = dist_to_finish / wheel_speed
                self.brake_time = time_since_start + brake_in

        if self.brake_time < time_since_start:
            self.speed_set_point = 0.

            # if car stopped, set self.finished to True
            if self.stopped_time is None and wheel_speed <= 0.1:
                self.stopped_time = time_since_start

        if self.stopped_time is not None and self.stopped_time + 1. < time_since_start:
            self.finished = True

        # 2. controls
        delta, _, log = stanley_steering(path, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {
            "time_since_start": time_since_start, 
            "brake_time": self.brake_time,
            "speed_setpoint": self.speed_set_point,
            "stopped_time": self.stopped_time,
            "finished": self.finished
        }

        return self.finished, delta, self.speed_set_point, debug_dict, path
