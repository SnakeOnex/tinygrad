import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import time
import sys
import math
from algorithms.steering import stanley_steering
from nodes.can1_node import Can1RecvItems, Can1SendItems


class Acceleration():
    ID = "Acceleration"

    def __init__(self, perception_out, can1_recv_state):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_state = can1_recv_state

        # CONTROLS CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5

        self.speed_set_point = 10.

        ## mission planning variables
        self.detecting_finish = False
        self.distance_to_finish = float('inf')
        self.stop_timestamp = None
        self.time_until_stop = None

        self.loop_count = 0

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

        # 2. planning
        cone_preds = world_state["world_preds"]

        big_cone_idxs = np.where(cone_preds[:,2] == 3)[0]
        self.loop_count += 1
        if len(big_cone_idxs) >= 1:
            self.detecting_finish = True
            self.distance_to_finish = np.mean(cone_preds[big_cone_idxs,1])
            self.stop_timestamp = time.perf_counter() + self.distance_to_finish / wheel_speed
        elif self.stop_timestamp is not None and time.perf_counter() - self.stop_timestamp >= 0.:
            self.speed_set_point = 0.

        if self.stop_timestamp is not None:
            self.time_until_stop = min(time.perf_counter() - self.stop_timestamp, 0.)

        # 2. controls
        delta, _, log = stanley_steering(
            world_state["path"], wheel_speed, self.linear_gain, self.nonlinear_gain)

        return delta, self.speed_set_point, {"detecting_finish": self.detecting_finish, "finish_dist": self.distance_to_finish, "time_until_stop": self.time_until_stop}
