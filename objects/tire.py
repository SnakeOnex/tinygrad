from ursina import *

class Tire(Entity):
    def __init__(self, **kwargs):
        super().__init__()

        self.model = "models/wheel.stl"
        # self.world_scale = (1000., 1000., 1000.)
        self.scale = (1000., 1000., 1000.)
        # self.scale = (1., 1., 1.)
        self.color = color.black

        for key, value in kwargs.items():
            setattr(self, key, value)
