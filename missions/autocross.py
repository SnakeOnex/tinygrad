import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math

from nodes.can1_node import Can1RecvItems, Can1SendItems


class Autocross():
    def __init__(self, perception_out, can1_recv_state):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_state = can1_recv_state

        # CONTROLLER CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5

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
        delta, _, log = self.stanley_steering(
            world_state["path"], wheel_speed, self.linear_gain, self.nonlinear_gain)

        return delta, 5.

    def stanley_steering(self, path, speed, gain, lateralGain, max_range=22.5):
        index = len(path)-1
        if index > 10:
            index = 10

        index = min(1, len(path)-1)

        targ = path[index]
        dx = targ[1]
        dy = targ[0]
        direction = math.atan2(dy, dx)
        if len(path) > 1:
            latOffset = path[1][0]
        else:
            latOffset = 0
        if speed > 0.3:
            nonLinear = 2 * math.atan2(lateralGain * latOffset, speed) / np.pi
        else:
            nonLinear = 0
            print("Not using non-linear")

        linear = gain * direction
        delta = linear + nonLinear
        delta *= 180/np.pi
        delta = np.clip(delta, -max_range, max_range)

        log_message = {"linear": linear,
                       "nonlinear": nonLinear,
                       "lateral_offset": latOffset,
                       "direction": direction,
                       "delta": delta
                       }
        return delta, index, log_message
