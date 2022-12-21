import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import math
import sys

from nodes.can1_node import Can1RecvItems, Can1SendItems
from config import path_planner_opt
from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner


class Skidpad():
    ID = "Skidpad"

    def __init__(self):
        mp.Process.__init__(self)

        # CONTROLLER CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        self.path_planner = PathPlanner(path_planner_opt)

        # SKIPAD WAYPOINT CONFIGURATION
        self.waypoint_state = False  #  is false when waypoints haven't been initialized
        self.waypoints = {
            "center": [False, (0, 0, 0)],
            "left_close": [False, (0, 0, 0)],
            "left_far": [False, (0, 0, 0)],
            "right_close": [False, (0, 0, 0)],
            "right_far": [False, (0, 0, 0)],
        }

    def loop(self, world_state, wheel_speed):
        """
        args:
          vision output (path, cone positions)
        ret:
          steering_angle
          wheelspeed_setpoint
        """
        # 1. receive perception data
        path = self.path_planner.find_path(world_state)
        # consider moving wheel speed to mission node
        delta, _, log = stanley_steering(
            path, wheel_speed, self.linear_gain, self.nonlinear_gain)

        return False, delta, 5., log, path
