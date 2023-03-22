import multiprocessing as mp
import time
import numpy as np
from sklearn.cluster import KMeans

from config import path_planner_opt
from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner
from algorithms.general import get_big_orange_distance, get_orange_centerline, local_to_global, global_to_local
from algorithms.skidpad_utils import *

CONE_CLASSES = {"yellow": 0.,
                "blue": 1.,
                "orange": 2.,
                "big": 3.}


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
        self.speed_set_point = 5.
        self.finished = False
        self.start_time = None
        self.finished_time = None

        #  SKIDPAD ALGORITHM VALUES
        self.keep_straight = True
        self.recorded_big_orange_cones = np.empty(shape=(1, 2))
        self.map_center_estimator = KMeans(n_clusters=2)
        self.heading = None
        self.glob_coords = None
        self.circles = []
        self.circle_centers = []
        self.estimated_cone_centers = []
        self.current_path = None
        self.center_pass_counter = 0
        self.last_passed_center = 0
        self.start_position = np.zeros((1, 2))

    def loop(self, **kwargs):
        """
        args:
          vision output (path, cone positions)
        ret:
          steering_angle
          wheelspeed_setpoint
        """

        percep_data = kwargs["percep_data"]
        wheel_speed = kwargs["wheel_speed"]

        self.glob_coords = kwargs["position"]
        self.heading = kwargs["euler"][0] + 180.

        if not self.start_time:
            self.start_time = time.time()

        time_since_start = time.time() - self.start_time

        if not self.start_position.any():
            self.start_position = kwargs["position"]

        # * big orange cone position estimation
        if self.keep_straight:
            if np.count_nonzero(percep_data[:, 2] == CONE_CLASSES["big"]) > 0:
                self.process_big_cones(percep_data.copy())
                self.estimate_map_position()
            else:
                self.estimate_map_position()
                self.current_path = self.circles["right"]
                self.keep_straight = False

        if self.keep_straight or self.finish_detect:
            percep_data = percep_data[percep_data[:, 2] == CONE_CLASSES["big"]] if self.keep_straight else percep_data[percep_data[:, 2] == CONE_CLASSES["orange"]]
            path = get_orange_centerline(percep_data)
            delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

            if self.finish_detect:
                percep_data[:, 2] = CONE_CLASSES["big"]
                dist_to_finish = get_big_orange_distance(cone_preds=percep_data, min_big_cones=4)

                if dist_to_finish is not None:
                    brake_in = dist_to_finish / wheel_speed
                    if brake_in < time.time() - self.last_passed_center:
                        self.speed_set_point *= 0.8

                if wheel_speed <= 0.1:
                    self.stopped_time = time.time()
                    self.finished = True

        else:
            if time.time() - self.last_passed_center > 5:
                self.check_passed_through_center()
            path = self.get_circular_path()
            delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {
            "Glob coords": self.glob_coords,
            "Time since start": time_since_start,
            "Finish time": self.finished_time,
            "Start pos": self.start_position,
            "Euler CAN heading": self.heading
            # "circle_centers": self.circle_centers
            # "cone_centers": self.estimated_cone_centers
        }

        return self.finished, delta, self.speed_set_point, debug_dict, path, controller_log["target"]

    def get_circular_path(self):
        """
        Computes circular path

        Returns:
            np.ndarray: 2d points making up the path curve
        """
        path = global_to_local(self.glob_coords, self.heading, self.current_path.copy())
        dists = np.sqrt(path[:, 0]**2 + path[:, 1]**2)
        path = path[np.argsort(dists, axis=0)]
        return path[path[:, 0] > 0][:5, :]

    def check_passed_through_center(self):
        """
        Checks whether car has passed through the center for the map and performs corresponding action
        """
        map_center = np.sum(self.estimated_cone_centers, axis=0) / 2
        if np.linalg.norm(self.glob_coords - map_center) <= Skidpad.WAYPOINT_RADIUS_SQ:
            if self.center_pass_counter == 2:
                self.current_path = self.circles["left"]
            elif self.center_pass_counter == 4:
                self.finish_detect = True
            self.center_pass_counter += 1
            self.last_passed_center = time.time()

    def process_big_cones(self, percep_data):
        """
        Converts detections of big orange cones from local to global coordinates and saves them
        Args:
            percep_data (numpy.ndarray): array of all cone detections in local coordinates
        """
        big_cones = percep_data[percep_data[:, 2] == CONE_CLASSES["big"]]
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
        a, b = np.sum(centers, axis=0) / 2, centers[0]
        D = np.linalg.norm(a - b)
        c1 = (1 - (d / D)) * a + (d / D) * b
        c2 = (1 + (d / D)) * a - (d / D) * b
        circle_centers = np.array([c1, c2])
        local_centers = global_to_local(self.start_position, 90, circle_centers.copy())
        circle_centers = circle_centers[np.lexsort((local_centers[:, 1], local_centers[:, 0]))]
        left = np.array([np.array([np.cos(pt) * d, np.sin(pt) * d]) + circle_centers[0] for pt in np.linspace(0, 2 * np.pi, num=50)])
        right = np.array([np.array([np.cos(pt) * d, np.sin(pt) * d]) + circle_centers[1] for pt in np.linspace(0, 2 * np.pi, num=50)])
        self.circle_centers = circle_centers
        self.circles = {"right": right, "left": left}
