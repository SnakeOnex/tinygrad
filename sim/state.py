import numpy as np
import json

from .math_helpers import angle_to_vector, global_to_local, rotate_around_point, filter_occluded_cones
# from cones.geometry_functions import filter_occluded_cones

class State():
    def __init__(self, mission, map_filepath):

        ## CAR PARAMS
        self.wheel_base = 1.5 # meters
        self.steering_speed = 90. # degrees per second
        self.max_steering_angle = 60. # max and min steering angle

        self.max_engine_force = 4000. # nm
        self.max_brake_force = 400. # nm
        self.drag_coef = 4
        self.rr_coef = self.drag_coef * 30
        self.mass = 200. # kg
        self.rotation_vector = np.array((0,0))

        self.car_width = 1.3
        self.car_length = 3.

        ## CAR STATE
        self.tson = True
        self.go_signal = 0

        self.speed_set_point = 0.

        ### CAR SENSORS
        self.occlusion_profile = [-6., 6., 2.5, 15.]

        self.mission = mission
        self.load_map(map_filepath)

    def update_state(self, timedelta):

        # CONTROLS
        if self.speed_set_point > self.speed:
            self.forward()
        elif self.speed_set_point == 0.:
            self.brake()

        self.handle_controls(timedelta)

        ## LONGITUDINAL
        F_traction = self.engine_force # tractive force
        F_drag = -self.drag_coef * self.speed**2 # air resistance
        F_rr = -self.rr_coef * self.speed # rolling resistance
        F_long = F_traction + F_drag + F_rr # longitudinal force
        self.car_pos -= self.rotation_vector
        acc = F_long / self.mass # acceleration
        self.speed += acc * timedelta

        ## LATERAL
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


    def handle_controls(self, timedelta):
        if self.steering_control == "LEFT":
            self.steering_angle += timedelta * self.steering_speed

            if self.steering_angle >= self.max_steering_angle:
                self.steering_angle = self.max_steering_angle
        elif self.steering_control == "RIGHT":
            self.steering_angle -= timedelta * self.steering_speed

            if self.steering_angle <= -self.max_steering_angle:
                self.steering_angle = -self.max_steering_angle

        elif self.steering_control == "NEUTRAL":
            if self.steering_angle > 0.15:
                self.steering_angle -= timedelta * self.steering_speed
            elif self.steering_angle < -0.15 :
                self.steering_angle += timedelta * self.steering_speed
            else:
                self.steering_angle = 0

        if self.traction_control == "FORWARD":
            self.engine_force = self.max_engine_force
        elif self.traction_control == "BRAKE":
            self.engine_force = -self.max_brake_force

            if self.speed <= 0.1:
                self.speed = 0.
                self.engine_force = 0.

        elif self.traction_control == "NEUTRAL":
            self.engine_force = 0.

        self.steering_control = "NEUTRAL"
        self.traction_control = "NEUTRAL"

    def load_map(self, map_filepath):
        with open(map_filepath, 'r') as f:
            self.map_dict = json.load(f)

        ## MAP STATE
        self.yellow_cones = np.array(self.map_dict["yellow_cones"]).reshape(-1,2)
        self.blue_cones = np.array(self.map_dict["blue_cones"]).reshape(-1,2)
        self.orange_cones = np.array(self.map_dict["orange_cones"]).reshape(-1,2)
        self.big_cones = np.array(self.map_dict["big_cones"]).reshape(-1,2)
        self.start_line = np.array(self.map_dict["start_line"]).reshape(-1,2)
        self.finish_line = np.array(self.map_dict["finish_line"]).reshape(-1,2)

        yc = np.hstack((self.yellow_cones, np.full((self.yellow_cones.shape[0], 1), 0)))
        bc = np.hstack((self.blue_cones, np.full((self.blue_cones.shape[0], 1), 1)))
        oc = np.hstack((self.orange_cones, np.full((self.orange_cones.shape[0], 1), 2)))
        boc = np.hstack((self.big_cones, np.full((self.big_cones.shape[0], 1), 3)))
        self.cones_world = np.vstack((yc, bc, oc, boc))

        self.car_pos = np.array(self.map_dict["car_position"])
        self.heading = self.map_dict["car_heading"]
        # self.discipline = self.map_dict["discipline"]

        ## CAR STATE
        self.reset_state()

    def get_detections(self):
        cones_local = global_to_local(np.array(self.cones_world[:, 0:2]), self.car_pos, self.heading)
        cones_local = np.hstack((cones_local, self.cones_world[:, 2:3]))

        ## occlusion profile
        cones_local = filter_occluded_cones(cones_local, self.occlusion_profile)

        cones_local[:, 0] *= -1

        return cones_local

    def get_can1_state(self):
        wheel_speed = self.speed
        steering_actual = self.steering_angle

        return (wheel_speed, steering_actual)

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
