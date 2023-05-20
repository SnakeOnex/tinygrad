import numpy as np
import time
import multiprocessing as mp
from config import inspection_config


class Inspection():
    ID = "Inspection"

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


    def loop(self, **kwargs):
        """
        inputs unused, loops 2 times the period of a sine function to steer wheels, motors set at stable m/s
        details in config.inspection_config
        Implemented based on FSG 2023 official rulebook
        """

       #  get current duration of inspection
        time_since_start = time.perf_counter() - self.start_timestamp
        # finish if reached goal time
        if time_since_start >= self.inspection_duration:
            self.steering_angle = 0.
            self.speed_set_point = 0.
            self.finished = True
        else:
            # else compute the steering angle
            steering_state = time_since_start * (self.n_periods * (2 * np.pi) / self.inspection_duration)
            self.steering_angle = np.sin(steering_state) * self.max_steering_angle
        torque = self.torque_set_point
        debug_dict = {
            "time_since_start": time_since_start,
            "completed %": (time_since_start / self.inspection_duration) * 100,
            "speed_setpoint": self.speed_set_point,
            "steering_angle": self.steering_angle
        }

        return self.finished, self.steering_angle, self.speed_set_point, torque, debug_dict, np.array([[0, 0]]), np.array([0, 0])
