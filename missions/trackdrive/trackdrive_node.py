import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math

from nodes.can1_node import Can1RecvItems, Can1SendItems

class Trackdrive(mp.Process):
    def __init__(self, perception_out, can1_recv_name, can1_send_name):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_name = can1_recv_name
        self.can1_send_name = can1_send_name

        ## CONTROLLER CONFIGURATION
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        # lookahead_dist = 3.3

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(name=self.can1_recv_name)
        self.can1_send_state = shared_memory.ShareableList(name=self.can1_send_name)
        self.path_sharemem = shared_memory.ShareableList([0. for _ in range(2*5)], name="path")

    def run(self):
        self.initialize()

        while True:
            # 1. receive perception data
            percep_data = self.perception_out.get()

            wheel_speed = self.can1_recv_state[Can1RecvItems.wheel_speed.value]
            path = percep_data["path"]

            print("TRACKDRIVE: PATH: ", path)

            for i in range(min(path.shape[0], 5)):
                self.path_sharemem[i] = float(path[i,0])
                self.path_sharemem[i+path.shape[0]-1] = float(path[i,1])

            delta, _, log = self.stanley_steering(path, wheel_speed, self.linear_gain, self.nonlinear_gain)

            print("TRACKDRIVE: CONROLLER: ", log)

            self.can1_send_state[Can1SendItems.steering.value] = float(delta)
            
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

        log_message = { "linear" : linear,
                        "nonlinear" : nonLinear,
                        "lateral_offset": latOffset,
                        "direction" : direction,
                        "delta" : delta
                        }
        return delta, index, log_message


