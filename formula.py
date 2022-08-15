from ursina import *
import numpy as np

class Formula(Entity):
    def __init__(self, **kwargs):
        super().__init__()
        self.model = 'cube'
        self.color = color.orange
        self.scale = (0.8,0.4,0.4)
        self.texture = 'white_cube',
        self.position = (1,1,1)
        self.rotation = (0,0,0)

        self.throttle = 0.
        self.velocity = (0.,0.,0.)

        self.turn = 2.

        for key, value in kwargs.items():
            setattr(self, key, value)

    def forward(self):
        self.throttle = min(1., self.throttle + 0.05)

    def left(self):
        self.rotation -= (0.,self.turn,0.)

        if self.rotation[1] >= 360:
            self.rotation = (0,0,0)

        if self.rotation[1] < 0:
            self.rotation = (0,359,0)

    def right(self):
        self.rotation += (0.,self.turn,0.)

        if self.rotation[1] >= 360:
            self.rotation = (0,0,0)

        if self.rotation[1] < 0:
            self.rotation = (0,359,0)


    def update(self):
        heading_pos = self.get_heading_pos()
        self.position += 0.08 * self.throttle * heading_pos
        self.throttle = max(0., self.throttle - 0.02)

    def get_heading_pos(self):
        heading_pos = np.array([0.,1.])
        heading = np.deg2rad(self.rotation[1])

        R = np.array([[np.cos(heading), np.sin(-heading)],
                     [np.sin(heading), np.cos(heading)]])

        heading_pos = R @ heading_pos
        heading_pos /= np.sum(np.abs(heading_pos))
        return Vec3(heading_pos[1], 0., heading_pos[0])
