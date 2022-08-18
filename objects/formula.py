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
        self.shader=lit_with_shadows_shader
        self.cam = False

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

        # CONFIGURATION
        self.wheel_base = 1.5

        ## STEERING
        self.steering_speed = 180. # degrees per second
        self.max_steering_angle = 60. # max and min steering angle

        ## TRACTION
        self.max_engine_force = 400. # nM
        self.max_brake_force = 2000.
        self.drag_coef = 0.4
        self.rr_coef = self.drag_coef * 30
        self.mass = 200.

        ## CAR STATE
        self.position = (0.,0.,0.)
        self.heading = 0.
        self.steering_angle = 0.
        self.speed = 0.
        self.engine_force = 0.

        self.debug_text = Text(origin=(2.5, -3.), font='VeraMono.ttf')

        for key, value in kwargs.items():
            setattr(self, key, value)

    def reset_state(self):
        self.position = (0.,0.,0.)
        self.heading = 0.
        self.steering_angle = 0.
        self.speed = 0.
        self.engine_force = 0.

    def forward(self):
        self.engine_force = self.max_engine_force

    def neutral(self):
        self.engine_force = 0.

    def brake(self):
        if self.speed > 0.:
            self.engine_force = -self.max_brake_force
        else:
            self.engine_force = 0.
            self.speed = 0.

    def left(self):
        self.steering_angle += time.dt * self.steering_speed

        if self.steering_angle >= self.max_steering_angle:
            self.steering_angle = self.max_steering_angle

    def right(self):
        self.steering_angle -= time.dt * self.steering_speed

        if self.steering_angle <= -self.max_steering_angle:
            self.steering_angle = -self.max_steering_angle


    def update(self):
        self.debug_text.text = ""
        self.rotation = self.real_rot + self.offset_rot
        # self.heading = self.steering_angle
        heading_vec = self.get_direction_vec(self.heading)
        steering_vec = self.get_direction_vec(self.steering_angle)

        ## LONGITUDINAL FORCE
        F_traction = self.engine_force # tractive force
        F_drag = -self.drag_coef * self.speed**2 # air resistance
        F_rr = -self.rr_coef * self.speed # rolling resistance
        F_long = F_traction + F_drag + F_rr # longitudinal force

        acc = F_long / self.mass
        self.speed += acc * time.dt

        velocity = heading_vec * self.speed

        ## LATERAL 
        turn_radius = self.wheel_base / np.sin(np.deg2rad(self.steering_angle))
        speed = np.sqrt(np.linalg.norm(velocity))
        rotation = speed / turn_radius
        self.heading += time.dt * np.rad2deg(rotation)
        self.real_rot.z = self.heading

        def num2str(num, pad_to=7):
            num_str = f"{num:.2f}"

            num_str = ((pad_to - len(num_str)) * " ") + num_str
            return num_str

        self.debug_text.text += f"F_long: {num2str(F_long)}\nAcc: {num2str(acc)}\nSpeed: {num2str(self.speed)}\n"

        self.position += time.dt * velocity

        # relative positions of things
        self.driver_pos = self.position - 1. * heading_vec + (0., 0.7 ,0.)

        if self.cam:
            camera.position = self.driver_pos
            camera.rotation = (0.,-self.heading,0.)

        self.fl_wheel.rotation = (0. , 0., self.steering_angle)
        self.fr_wheel.rotation = (0. , 0., self.steering_angle)

        # debugging text
        self.debug_text.text += f"heading: {num2str(self.heading)}\nsteering_angle: {num2str(self.steering_angle)}"

    def get_direction_vec(self, angle):
        vec = np.array([0.,1.])
        angle = np.deg2rad(angle)

        R = np.array([[np.cos(angle), np.sin(-angle)],
                     [np.sin(angle), np.cos(angle)]])

        vec = R @ vec
        vec /= np.linalg.norm(vec)

        return np.array((vec[0], 0., vec[1]))

