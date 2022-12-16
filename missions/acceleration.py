import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math
from algorithms.steering import stanley_steering
from config import path_planner_opt
from algorithms.path_planning import PathPlanner
from nodes.can1_node import Can1RecvItems, Can1SendItems



class Acceleration():
    ID = "Acceleration"

    def __init__(self, perception_out, can1_recv_state):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_state = can1_recv_state

        # CONTROLLER CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        self.path_planner = PathPlanner(path_planner_opt)

    def loop(self, world_state):
        """
        args:
          vision output (path, cone positions)
        ret:
          steering_angle
          wheelspeed_setpoint
        """
        # 1. receive perception data
        wheel_speed = self.can1_recv_state[Can1RecvItems.wheel_speed.value]
        path = self.path_planner.find_path(world_state)
        # consider moving wheel speed to mission node
        delta, _, log = stanley_steering(
            path, wheel_speed, self.linear_gain, self.nonlinear_gain)

        return delta, 5., log, path
