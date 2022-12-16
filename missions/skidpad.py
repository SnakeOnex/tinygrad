import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
from algorithms.steering import stanley_steering
import math
from nodes.can1_node import Can1RecvItems, Can1SendItems


class Skidpad():
    ID = "Skidpad"

    def __init__(self, perception_out, can1_recv_state):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_state = can1_recv_state

        # CONTROLLER CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5

        # SKIPAD WAYPOINT CONFIGURATION
        self.waypoint_state = False  #  is false when waypoints haven't been initialized
        self.waypoints = {
            "center": [False, (0, 0, 0)],
            "left_close": [False, (0, 0, 0)],
            "left_far": [False, (0, 0, 0)],
            "right_close": [False, (0, 0, 0)],
            "right_far": [False, (0, 0, 0)],
        }

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
        # consider moving wheel speed to mission node
        delta, _, log = stanley_steering(
            world_state["path"], wheel_speed, self.linear_gain, self.nonlinear_gain)

        return delta, 5., log
