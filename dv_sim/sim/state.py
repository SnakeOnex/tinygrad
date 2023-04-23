import numpy as np
import json
from enum import IntEnum
from scipy.integrate import odeint
from .physics_models import kinematic_model, single_track_model
from .math_helpers import angle_to_vector, global_to_local, rotate_around_point, filter_occluded_cones
# from cones.geometry_functions import filter_occluded_cones

class AS(IntEnum):
    OFF = 0
    READY = 1
    DRIVING = 2
    FINISHED = 3
    EMERGENCY = 4

class State():
    def __init__(self, mission, map_filepath):

        self.dotPsi = 0.0
        self.dRhoF = 0.0
        self.dRhoR = 0.0
        self.beta = 0.0
        self.manual = False
        self.heading = 0.0
        ## CAR PARAMS
        self.wheel_base = 1.5 # meters
        self.steering_speed = 90 # degrees per second
        self.max_steering_angle = 25. # max and min steering angle

        self.max_engine_force = 3000. # nm
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
        self.emergency_signal = 0
        self.AS = AS.OFF
        self.model_switched = False
        self.speed_set_point = 0.
        self.steering_angle_set_point = 0.
        self.velocity = np.array([0., 0.])

        ### CAR SENSORS
        self.occlusion_profile = [-6., 6., 2.5, 15.]

        self.mission = mission
        self.load_map(map_filepath)

    def update_state(self, timedelta):

        # CONTROLS
        if self.speed_set_point > self.speed and not self.manual:
            self.forward()
        elif self.speed_set_point == 0. and not self.manual:
            self.brake()

        if self.emergency_signal == 1:
            self.brake()

        if not self.manual:
            if self.steering_angle_set_point > self.steering_angle:
                self.steer_left()
            else:
                self.steer_right()

        self.handle_controls(timedelta)

        ## LONGITUDINAL
        F_traction = self.engine_force # tractive force
        F_drag = -self.drag_coef * self.speed**2 # air resistance
        F_rr = -self.rr_coef * self.speed # rolling resistance
        F_long = F_traction + F_drag + F_rr # longitudinal force
        
        acc = F_long / self.mass # acceleration

        if self.speed < 0.5:
            self.speed += acc * timedelta
            states = [self.car_pos[0],self.car_pos[1],np.deg2rad(self.heading)]
            tspan = [0.0,timedelta]
            out = odeint(kinematic_model,states,tspan,args=(self.speed,np.deg2rad(self.steering_angle)))
            self.car_pos[0] = out[1][0]
            self.car_pos[1] = out[1][1]
            self.heading = np.rad2deg(out[1][2])
            self.model_switched = True
        else:
            if self.model_switched:
                self.dRhoF = self.speed/0.2
                self.dRhoR = self.speed/0.2
                self.model_switched = False
            z0 = [self.speed, self.beta, self.dRhoR, self.dRhoF, self.dotPsi, np.deg2rad(self.heading),
                  self.car_pos[0], self.car_pos[1]]
            tspan = [0.0, timedelta]
            #print(self.tauF,self.tauR)
            z = odeint(single_track_model, z0, tspan, args=(np.deg2rad(self.steering_angle), self.tauF, self.tauR))
            self.speed = z[1][0]
            self.beta = z[1][1]
            self.dRhoR = z[1][2]
            self.dRhoF = z[1][3]
            self.dotPsi = z[1][4]
            self.heading = np.rad2deg(z[1][5])
            x = z[1][6]
            y = z[1][7]
            self.car_pos = np.array([x, y])



        if self.heading >= 360.:
            self.heading -= 360
        if self.heading < 0.:
            self.heading += 360

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
            if self.steering_angle - timedelta * self.steering_speed > 0.001:
                self.steering_angle -= timedelta * self.steering_speed
            elif self.steering_angle + timedelta * self.steering_speed < -0.001 :
                self.steering_angle += timedelta * self.steering_speed
            else:
                self.steering_angle = 0

        # print(self.traction_control)
        if self.traction_control == "FORWARD":
            self.engine_force = self.max_engine_force
            self.tauF = 10
            self.tauR = 10
        elif self.traction_control == "BRAKE":
            self.engine_force = -self.max_brake_force
            self.tauF = -10
            self.tauR = -10
            if self.speed <= 0.1:
                self.speed = 0.
                self.engine_force = 0.

        elif self.traction_control == "NEUTRAL":
            self.engine_force = 0.
            self.tauF = 0
            self.tauR = 0
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
