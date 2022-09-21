import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math
import time

from nodes.can1_node import Can1RecvItems, Can1SendItems

class Mission(mp.Process):
    def __init__(self, perception_out, can1_recv_name, can1_send_name):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_name = can1_recv_name
        self.can1_send_name = can1_send_name

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(name=self.can1_recv_name)
        self.can1_send_state = shared_memory.ShareableList(name=self.can1_send_name)
        self.path_sharemem = shared_memory.ShareableList([0. for _ in range(2*5)], name="path")

    def run(self):
        self.initialize()

        while True:
        	start_time = time.perf_counter()

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
            
