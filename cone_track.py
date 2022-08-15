import numpy as np
from pathlib import Path

class ConeTrack():
    def __init__(self, path):
        self.load_from_npy(path)

    def load_from_npy(self, path):
        cones = np.load(path)

        self.yellow_cones = cones[cones[:,2] == 0, :2].reshape(-1,2)
        self.blue_cones = cones[cones[:,2] == 1, :2].reshape(-1,2)
        self.orange_cones = cones[cones[:,2] == 2, :2].reshape(-1,2)
        self.big_cones = cones[cones[:,2] == 3, :2].reshape(-1,2)

    def load_from_json(self):
        pass

