from ursina import *
from ursina.shaders import lit_with_shadows_shader


class Cone(Entity):
    def __init__(self, **kwargs):
        super().__init__()

        self.scale = (0.00011, 0.00011, 0.00011)
        self.model = 'models/yellow_cone.obj'
        self.y_pos = 0.01
        self.shader = lit_with_shadows_shader
        self.texture = 'models/yellow_cone.mtl'

        for key, value in kwargs.items():
            setattr(self, key, value)
