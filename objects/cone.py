from ursina import *
from ursina.shaders import lit_with_shadows_shader


class Cone(Entity):
    def __init__(self, **kwargs):
        super().__init__()
        # self.model = 'cube'
        # self.collider = 'box'
        self.scale = (0.00011, 0.00011, 0.00011)
        self.y_pos = 0.01
        self.shader = lit_with_shadows_shader

        for key, value in kwargs.items():
            setattr(self, key, value)
