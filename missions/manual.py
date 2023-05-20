import numpy as np
import time
import multiprocessing as mp
from config import inspection_config


class Manual():
    ID = "Manual"

    def __init__(self):
        mp.Process.__init__(self)
        self.start_timestamp = time.perf_counter()
        self.inspection_duration = inspection_config["duration"]
        self.speed_set_point = inspection_config["speed_set_point"]
        self.max_steering_angle = inspection_config["max_steering_angle"]
        self.n_periods = inspection_config["n_periods"]
        self.finished = False
        self.steering_angle = 0.
        self.torque_set_point = 5.
        self.speed_set_point = 10.


    def loop(self, **kwargs):
        """
        inputs unused, loops 2 times the period of a sine function to steer wheels, motors set at stable m/s
        details in config.inspection_config
        Implemented based on FSG 2023 official rulebook
        """

        accelerator_pos = kwargs["accelerator_pos"]
        print("accelerator_pos", accelerator_pos)

        torque = (accelerator_pos / 100) * 20

        return self.finished, self.steering_angle, self.speed_set_point, torque, debug_dict, np.array([[0, 0]]), np.array([0, 0])
