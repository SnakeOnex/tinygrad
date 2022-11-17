from ursina import *

class SteeringWheel(Entity):
    def __init__(self, **kwargs):
        super().__init__()

        self.model = "models/steering_wheel.stl"
        # self.world_scale = (1000., 1000., 1000.)
        self.scale = (40., 40., 40.)
        # self.scale = (1., 1., 1.)
        self.color = color.black
        # self.texture = "concrete.jpg"
        # self.texture = "models/tire.jpg"
        self.texture_scale = (4,4)

        for key, value in kwargs.items():
            setattr(self, key, value)
