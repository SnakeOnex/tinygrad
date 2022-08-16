import numpy as np
from pathlib import Path

from ursina import *
from ursina.shaders import lit_with_shadows_shader

class ConeTrack():
    def __init__(self, path):
        self.load_from_npy(path)

        self.cone_model = 'models/cone_yellow.fbx'
        self.cone_scale = (0.1, 0.1, 0.1)
        self.texture = 'white_cube'
        self.y_pos = 0.01
        self.shader = lit_with_shadows_shader

    def load_from_npy(self, path):
        cones = np.load(path)

        self.yellow_cones = cones[cones[:,2] == 1, :2].reshape(-1,2)
        self.blue_cones = cones[cones[:,2] == 0, :2].reshape(-1,2)
        self.orange_cones = cones[cones[:,2] == 2, :2].reshape(-1,2)
        self.big_cones = cones[cones[:,2] == 3, :2].reshape(-1,2)

    def load_from_json(self):
        pass

    def render_cones(self):
        def render_cone(color, pos):
            Entity(
                model = self.cone_model,
                color = color,
                texture = self.texture,
                position = pos,
                scale = self.cone_scale,
                shader=self.shader
            )
            return

        [render_cone(color.yellow, (c[0], self.y_pos, c[1])) for c in self.yellow_cones]
        [render_cone(color.blue, (c[0], self.y_pos, c[1])) for c in self.blue_cones]
        [render_cone(color.orange, (c[0], self.y_pos, c[1])) for c in self.orange_cones]
        [render_cone(color.red, (c[0], self.y_pos, c[1])) for c in self.big_cones]
        

