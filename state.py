import numpy as np
import json

from math_helpers import angle_to_vector, rotate_around_point

class State():
    def __init__(self, map_filepath):
        with open(map_filepath, 'r') as f:
            self.map_dict = json.load(f)
        
        ## MAP STATE
        self.yellow_cones = np.array(self.map_dict["yellow_cones"]).reshape(-1,2)
        self.blue_cones = np.array(self.map_dict["blue_cones"]).reshape(-1,2)
        self.orange_cones = np.array(self.map_dict["orange_cones"]).reshape(-1,2)
        self.big_cones = np.array(self.map_dict["big_cones"]).reshape(-1,2)

        ## CAR PARAMS
        self.wheel_base = 1.5 # meters
        self.steering_speed = 90. # degrees per second
        self.max_steering_angle = 30. # max and min steering angle

        self.max_engine_force = 2000. # nm
        self.max_brake_force = 2000. # nm
        self.drag_coef = 4
        self.rr_coef = self.drag_coef * 30
        self.mass = 200. # kg
        self.rotation_vector = np.array((0,0))

        ## CAR STATE
        self.car_pos = np.array(self.map_dict["car_position"])
        self.heading = self.map_dict["car_heading"]
        self.steering_angle = 0.
        self.speed = 0.
        self.engine_force = 0.
        self.steering_control = "NEUTRAL" # "LEFT", "RIGHT", "NEUTRAL"
        self.traction_control = "NEUTRAL" # "FORWARD", "BREAK", "NEUTRAL"

    def update_state(self, timedelta):

        ## CONTROLS
        if self.steering_control == "LEFT":
            self.steering_angle += timedelta * self.steering_speed

            if self.steering_angle >= self.max_steering_angle:
                self.steering_angle = self.max_steering_angle

            self.steering_control = "NEUTRAL"
        elif self.steering_control == "RIGHT":
            self.steering_angle -= timedelta * self.steering_speed

            if self.steering_angle <= -self.max_steering_angle:
                self.steering_angle = -self.max_steering_angle

            self.steering_control = "NEUTRAL"
        elif self.steering_control == "NEUTRAL":
            if self.steering_angle > 0.15:
                self.steering_angle -= timedelta * self.steering_speed
            elif self.steering_angle < -0.15 :
                self.steering_angle += timedelta * self.steering_speed
            else:
                self.steering_angle = 0

        if self.traction_control == "FORWARD":
            self.engine_force = self.max_engine_force
            self.traction_control = "NEUTRAL"
        elif self.traction_control == "BRAKE":
            self.engine_force = -self.max_brake_force

            if self.speed <= 0.:
                self.speed = 0.
                self.engine_force = 0.

            self.traction_control = "NEUTRAL"
        elif self.traction_control == "NEUTRAL":
            self.engine_force = 0.

        # LONGITUDINAL
        F_traction = self.engine_force # tractive force
        F_drag = -self.drag_coef * self.speed**2 # air resistance
        F_rr = -self.rr_coef * self.speed # rolling resistance
        F_long = F_traction + F_drag + F_rr # longitudinal force
        self.car_pos -= self.rotation_vector
        acc = F_long / self.mass # acceleration
        self.speed += acc * timedelta 

        # LATERAL
        turn_radius = self.wheel_base / np.sin(np.deg2rad(self.steering_angle))
        rotation = self.speed / (turn_radius*3.6)
        self.heading += timedelta * np.rad2deg(rotation)
        self.rotation_vector = rotate_around_point(self.heading,(0,0),(-self.wheel_base,0))

        if self.heading >= 360.:
            self.heading -= 360
        if self.heading < 0.:
            self.heading += 360

        heading_vec = angle_to_vector(self.heading)
        velocity = heading_vec * self.speed
        self.car_pos += self.rotation_vector
        self.car_pos += timedelta * velocity
        
        

    def get_detections(self):
        pass

    def steer_left(self):
        self.steering_control = "LEFT"

    def steer_right(self):
        self.steering_control = "RIGHT"

    def forward(self):
        self.traction_control = "FORWARD"

    def brake(self):
        self.traction_control = "BRAKE"

    def reset_state(self):
        self.car_pos = np.array(self.map_dict["car_position"])
        self.heading = self.map_dict["car_heading"]
        self.steering_angle = 0.
        self.speed = 0.
        self.engine_force = 0.
        self.steering_control = "NEUTRAL" # "LEFT", "RIGHT", "NEUTRAL"
        self.traction_control = "NEUTRAL" # "FORWARD", "BREAK", "NEUTRAL"

