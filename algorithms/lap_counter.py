import numpy as np


class LapCounter:
    def __init__(self):
        self.lap_count = 0
        self.start_position = None
        self.left_start_area = False
        self.start_area = 1.0

    def update_lap_count(self, position):
        if self.start_position is not None:
            if not self.left_start_area and not self.is_in_area(position):
                self.left_start_area = True
            elif self.left_start_area and self.is_in_area(position):
                self.left_start_area = False
                self.lap_count += 1

    def is_in_area(self, position: np.ndarray):
        return np.linalg.norm(self.start_position - position) < self.start_area

    def set_start_position(self, position):
        self.start_position = position

    def reset(self):
        self.lap_count = 0
        self.start_position = None
        self.left_start_area = False
        self.start_area = 1.0
