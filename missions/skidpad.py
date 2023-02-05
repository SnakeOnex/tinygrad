import multiprocessing as mp
# ! temporary
import zmq
import pickle
import numpy as np
from sklearn.cluster import KMeans

from config import path_planner_opt
from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner

CONE_CLASSES = {"yellow": 0.,
                "blue": 1.,
                "orange": 2.,
                "big": 3.}

# ! temporary array for sim testing
SIM_WAYPOINTS = {'start_left': np.array([-3.18173506,  3.12047806]),
                 'start_right': np.array([3.32004047, 3.02806534]),
                 'end_left': np.array([-3.32004047, 16.97193466]),
                 'end_right': np.array([3.18173506, 16.87952194]),
                 'center': np.array([0., 8.])}


class Skidpad():
    ID = "Skidpad"
    WAYPOINT_RADIUS_SQ = 1.5
    RING_RADIUS = 9.215

    def __init__(self):
        mp.Process.__init__(self)

        # CONTROLLER CONFIGURATION
        self.lookahead_dist = 3.
        self.linear_gain = 2.05
        self.nonlinear_gain = 1.5
        self.path_planner = PathPlanner(path_planner_opt)
        self.finish_detect = False

        self.speed_set_point = 3.
        self.finished = False

        # SKIPAD WAYPOINT CONFIGURATION
        self.waypoint_state = False  #  is false when waypoints haven't been initialized
        self.waypoints = {key: [SIM_WAYPOINTS[key], False, 0]
                          for key in SIM_WAYPOINTS.keys()}

        self.cone_boolean_mask = "all"
        self.keep_straight = True
        self.waypoints["center"][1] = True
        self.recorded_big_orange_cones = np.empty(shape=(1, 2))
        self.map_center_estimator = KMeans(n_clusters=2)
        self.heading = None
        self.glob_coords = None
        self.circles = []
        self.circle_centers = []
        self.estimated_cone_centers = []

        # ! temporary socket for receiving global coordinates
        context = zmq.Context()
        self.glob_coord_socket = context.socket(zmq.SUB)
        self.glob_coord_socket.connect("tcp://127.0.0.1:50004")
        self.glob_coord_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.glob_coord_poller = zmq.Poller()
        self.glob_coord_poller.register(self.glob_coord_socket, zmq.POLLIN)

    def loop(self, world_state, wheel_speed):
        """
        args:
          vision output (path, cone positions)
        ret:
          steering_angle
          wheelspeed_setpoint
        """
        # 0. get global coordinates
        while self.glob_coord_poller.poll(0.):
            data = pickle.loads(self.glob_coord_socket.recv())
            self.glob_coords = data[0]
            self.heading = data[1]
            self.update_waypoint_state(data[0])

        # * big orange cone position estimation
        if not self.waypoint_state:
            if np.count_nonzero(world_state[:, 2] == CONE_CLASSES["big"]) > 0:
                self.process_big_cones(world_state.copy())
                self.estimate_map_position()
            else:
                self.estimate_map_position()
                self.waypoint_state = True
                self.keep_straight = False

        # 1. receive perception data

        if self.keep_straight:
            path = self.path_planner.find_path(world_state)
            delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)
            delta = 0.
        else:
            path = self.global_to_local(self.circles[0])
            delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {
            "heading": self.heading,
            "glob_coords": self.glob_coords,
            "circle_centers": self.circle_centers,
            "cone_centers": self.estimated_cone_centers
        }
        return self.finished, delta, self.speed_set_point, debug_dict, (path, controller_log["target"])

    def update_waypoint_state(self, coords):
        current_passed_zone = None
        for key, value in self.waypoints.items():
            if value[1]:
                if np.sum(np.square(coords-value[0])) <= Skidpad.WAYPOINT_RADIUS_SQ:
                    current_passed_zone = key
                    break

        if current_passed_zone != None:
            n_passes = self.waypoints[current_passed_zone][2]
            new_cone_mask = None
            next_active_zone = None
            if current_passed_zone == "center":
                if n_passes == 0:
                    self.keep_straight = False
                if n_passes == 0 or n_passes == 1:
                    new_cone_mask = "blue_only"
                    next_active_zone = "end_left"
                elif n_passes == 2 or n_passes == 3:
                    new_cone_mask = "yellow_only"
                    next_active_zone = "end_right"
                else:
                    new_cone_mask = "orange_only"
                    self.finish_detect = True
            elif current_passed_zone == "start_left":
                new_cone_mask = "no_blue"
                next_active_zone = "center"
            elif current_passed_zone == "start_right":
                new_cone_mask = "no_yellow"
                next_active_zone = "center"
            elif current_passed_zone == "end_left":
                new_cone_mask = "all"
                next_active_zone = "start_left"
            elif current_passed_zone == "end_right":
                new_cone_mask = "all"
                next_active_zone = "start_right"

            self.waypoints[current_passed_zone][2] += 1
            self.waypoints[current_passed_zone][1] = False
            self.waypoints[next_active_zone][1] = True
            self.cone_boolean_mask = new_cone_mask

    def filter_state(self, word_state):
        if self.cone_boolean_mask == "all":
            return word_state
        elif self.cone_boolean_mask == "blue_only":
            return word_state[(word_state[:, 2] == CONE_CLASSES["blue"])]
        elif self.cone_boolean_mask == "no_blue":
            return word_state[(word_state[:, 0] >= 0) & (word_state[:, 2] != CONE_CLASSES["blue"])]
        elif self.cone_boolean_mask == "no_yellow":
            return word_state[(word_state[:, 0] <= 0)]
        elif self.cone_boolean_mask == "yellow_only":
            return word_state[(word_state[:, 2] == CONE_CLASSES["yellow"]) & (word_state[:, 0] >= 0)]
        elif self.cone_boolean_mask == "orange_only":
            return word_state[word_state[:, 2] == CONE_CLASSES["orange"]]

    def global_to_local(self, path):
        """
        Doesn't work yet
        """
        car_heading = np.deg2rad(self.heading-90)
        R = np.array([[np.cos(car_heading), np.sin(car_heading)],
                      [-np.sin(car_heading), np.cos(car_heading)]])
        path -= self.glob_coords
        path = (R @ path.T).T
        path[:, 0] *= -1
        path = np.flip(path, axis=1)
        return path  # [path[:, 0] > 0]

    def process_big_cones(self, world_state):
        """
        Converts detections of big orange cones from local to global coordinates and saves them
        Args:
            world_state (numpy.ndarray): array of all cone detections in local coordinates
        """
        big_cones = world_state[world_state[:, 2] == CONE_CLASSES["big"]]
        big_cones_2d = np.flip(np.delete(big_cones, 2, axis=1), axis=1)
        big_cones_2d[:, 0] *= -1
        big_cones_2d += np.array(self.glob_coords)
        self.recorded_big_orange_cones = np.append(self.recorded_big_orange_cones, big_cones_2d, axis=0)

    def estimate_map_position(self):
        """
        Estimates the position of the two circles in global coordinates
        """
        self.map_center_estimator.fit(self.recorded_big_orange_cones)
        self.estimated_cone_centers = self.map_center_estimator.cluster_centers_
        self.estimate_map_circles(Skidpad.RING_RADIUS)

    def estimate_map_circles(self, d):
        """
        Estimates the centers of map circles and creates points for each circle
        Args:
            d (float): radius of circles to be generated
        """
        centers = self.estimated_cone_centers
        a, b = np.sum(centers, axis=0)/2, centers[0]
        D = np.linalg.norm(a-b)
        c1 = (1-(d/D)) * a + (d/D)*b
        c2 = (1+(d/D)) * a - (d/D)*b
        angles = np.linspace(0, 2*np.pi, num=50)
        coords1 = np.array([np.array([np.cos(pt)*d, np.sin(pt)*d]) + c1 for pt in angles])
        coords2 = np.array([np.array([np.cos(pt)*d, np.sin(pt)*d]) + c2 for pt in angles])
        self.circle_centers = [c1, c2]
        self.circles = [coords1, coords2]
