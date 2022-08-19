from ursina import *
from ursina.shaders import lit_with_shadows_shader


class Cone(Entity):
    def __init__(self, **kwargs):
        super().__init__()

        self.scale = (0.1, 0.1, 0.1)
        self.color = color.yellow
        self.model = 'models/cone_yellow.fbx'
        self.y_pos = 0.01
        self.shader = lit_with_shadows_shader

        for key, value in kwargs.items():
            setattr(self, key, value)
