import numpy as np
import json

from math_helpers import angle_to_vector

class State():
    def __init__(self, map_filepath):
        with open(map_filepath, 'r') as f:
            map_dict = json.load(f)
        
        ## MAP STATE
        self.yellow_cones = np.array(map_dict["yellow_cones"]).reshape(-1,2)
        self.blue_cones = np.array(map_dict["blue_cones"]).reshape(-1,2)
        self.orange_cones = np.array(map_dict["orange_cones"]).reshape(-1,2)
        self.big_cones = np.array(map_dict["big_cones"]).reshape(-1,2)

        ## CAR PARAMS
        self.wheel_base = 1.5 # meters
        self.steering_speed = 360. # degrees per second
        self.max_steering_angle = 60. # max and min steering angle

        self.max_engine_force = 400. # nm
        self.max_brake_force = 2000. # nm
        self.drag_coef = 0.4
        self.rr_coef = self.drag_coef * 30
        self.mass = 200. # kg

        ## CAR STATE
        self.car_pos = np.array(map_dict["car_position"])
        self.heading = map_dict["car_heading"]
        self.steer_angle = self.heading
        self.speed = 0.
        self.engine_force = 0.
        self.steering_control = "NEUTRAL" # "LEFT", "RIGHT", "NEUTRAL"
        self.traction_control = "NEUTRAL" # "FORWARD", "BREAK", "NEUTRAL"

    def update_state(self, timedelta):

        ## CONTROLS
        if self.steering_control == "LEFT":
            self.steering_angle -= timedelta * self.steering_speed
            self.steering_control = "NEUTRAL"
        elif self.steering_control == "RIGHT":
            self.steering_angle += timedelta * self.steering_speed
            self.steering_control = "NEUTRAL"

        if self.traction_control == "FORWARD":
            self.engine_force = self.max_engine_force
            self.traction_control = "NEUTRAL"
        elif self.traction_control == "BREAK":
            self.engine_force = self.max_brake_force
            self.traction_control = "NEUTRAL"
        elif self.traction_control == "NEUTRAL":
            self.engine_force = 0.

        # LONGITUDINAL
        F_traction = self.engine_force # tractive force
        F_drag = -self.drag_coef * self.speed**2 # air resistance
        F_rr = -self.rr_coef * self.speed # rolling resistance
        F_long = F_traction + F_drag + F_rr # longitudinal force

        acc = F_long / self.mass # acceleration
        self.speed += acc * timedelta 

        # LATERAL
        turn_radius = self.wheel_base / np.sin(np.deg2rad(self.steering_angle))
        rotation = self.speed / turn_radius
        self.heading += timedelta * np.rad2deg(rotation)

        heading_vec = angle_to_vector(self.heading)
        velocity = heading_vec * self.speed
        self.position += timedelta * velocity

    def steer_left(self):
        self.steering_control = "LEFT"

    def steer_right(self):
        self.steering_control = "RIGHT"

    def forward(self):
        self.traction_control = "FORWARD"

    def brake(self):
        self.traction_control = "BRAKE"

