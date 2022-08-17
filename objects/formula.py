from ursina import *
from ursina.shaders import lit_with_shadows_shader
import numpy as np

from .tire import Tire

class Formula(Entity):
    def __init__(self, **kwargs):
        super().__init__()
        self.model = 'models/whole_car.stl'
        self.scale = (0.001,0.001,0.001)
        self.color = color.orange
        self.position = (0.,0.,0.)
        self.shader=lit_with_shadows_shader

        # relative positions of things
        self.driver_pos = (0., 0., 0.)

        # rotate the model so things make sense
        self.offset_rot = Vec3(270., 0., 90.)

        # real rotation after the rotation offset
        self.real_rot = Vec3(0., 0., 0.)

        ## tires
        tire_color = color.black
        self.fl_wheel = Tire(position=(0., -550., 235.), parent=self)
        self.fr_wheel = Tire(position=(0., 550., 235.), parent=self)
        self.rl_wheel = Tire(position=(-1500., -550., 235.), parent=self)
        self.rr_wheel = Tire(position=(-1500., 550., 235.), parent=self)

        self.heading = 0.
        self.steering_angle = 0.
        self.fl_wheel.rotation = (0. ,0., self.steering_angle)
        self.fr_wheel.rotation = (0. ,0., self.steering_angle)

        # physics things
        self.throttle = 0.
        self.turn = 1.
        self.velocity = (0.,0.,0.)

        self.text = Text()

        for key, value in kwargs.items():
            setattr(self, key, value)

    def forward(self):
        self.throttle = min(1., self.throttle + 0.05)

    def left(self):
        # self.real_rot -= Vec3(0.,self.turn,0.)
        self.steering_angle += self.turn

        if self.rotation[1] >= 360:
            self.real_rot = Vec3(0., 0., 0.)

        if self.rotation[1] < 0:
            self.real_rot = Vec3(0., 359., 0.)

    def right(self):
        # self.real_rot += Vec3(0., self.turn, 0.)
        self.steering_angle -= self.turn

        if self.rotation[1] >= 360:
            self.real_rot = Vec3(0., 0., 0.)

        if self.rotation[1] < 0:
            self.real_rot = Vec3(0., 359., 0.)

    def update(self):
        self.rotation = self.real_rot + self.offset_rot
        # self.heading = self.steering_angle
        heading_pos = self.get_heading_pos()
        steering_pos = self.get_direction(self.steering_angle)

        self.position += 0.08 * self.throttle * steering_pos
        # self.position += 0.08 * self.throttle * heading_pos

        self.throttle = max(0., self.throttle - 0.02)

        # relative positions of things
        self.driver_pos = self.position - 1.21 * heading_pos + (0., 0.68 ,0.)

        self.fl_wheel.rotation = (0. , 0., self.steering_angle)
        self.fr_wheel.rotation = (0. , 0., self.steering_angle)

        # debugging text
        self.text.text = f"heading: {self.heading}\n heading_pos: {heading_pos} \n steering_angle: {self.steering_angle} \n ster_pos: {steering_pos}"

    def get_direction(self, angle):
        heading_pos = np.array([0.,1.])
        angle = np.deg2rad(self.rotation[1])

        R = np.array([[np.cos(angle), np.sin(-angle)],
                     [np.sin(angle), np.cos(angle)]])

        heading_pos = np.linalg.inv(R) @ heading_pos
        # heading_pos /= np.sum(np.abs(heading_pos))
        heading_pos /= np.linalg.norm(heading_pos)

        return Vec3(heading_pos[0], 0., heading_pos[1])

    def get_heading_pos(self):
        heading_pos = np.array([0.,1.])
        heading = np.deg2rad(self.rotation[1])

        R = np.array([[np.cos(heading), np.sin(-heading)],
                     [np.sin(heading), np.cos(heading)]])

        heading_pos = np.linalg.inv(R) @ heading_pos
        # heading_pos /= np.sum(np.abs(heading_pos))
        heading_pos /= np.linalg.norm(heading_pos)

        return Vec3(heading_pos[0], 0., heading_pos[1])

