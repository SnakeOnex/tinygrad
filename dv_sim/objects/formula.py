from ursina import *
from ursina.shaders import lit_with_shadows_shader
import numpy as np

from .tire import Tire
from .steering_wheel import SteeringWheel

class Formula(Entity):
    def __init__(self, **kwargs):
        super().__init__()
        self.model = 'models/whole_car.stl'
        # self.model = 'models/car.stl'
        # self.scale = (0.001,0.001,0.001)
        self.scale = (0.001, 0.001,0.001)
        self.color = color.orange
        self.shader=lit_with_shadows_shader

        # relative positions of things
        self.driver_pos = np.array([0., 0., 0.])

        # rotate the model so things make sense
        self.offset_rot = Vec3(270., 0., 0.)

        ## tires
        tire_color = color.black
        self.fl_wheel = Tire(position=(0., -550., 235.), parent=self)
        self.fr_wheel = Tire(position=(0., 550., 235.), parent=self)
        self.rl_wheel = Tire(position=(-1500., -550., 235.), parent=self)
        self.rr_wheel = Tire(position=(-1500., 550., 235.), parent=self)

        ## steering wheel
        self.steering_wheel = SteeringWheel(position=(-540., 0., 540.), parent=self)

        for key, value in kwargs.items():
            setattr(self, key, value)

