import multiprocessing as mp
import time
# ! temporary
import zmq
import pickle
import numpy as np
from sklearn.cluster import KMeans

from config import path_planner_opt
from algorithms.steering import stanley_steering
from algorithms.path_planning import PathPlanner
from algorithms.general import get_big_orange_distance, get_orange_centerline

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
        self.brake_time = float('inf')

        # SKIDPAD ALGORITHM VALUES
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
        if not self.start_time:
            self.start_time = time.time()

        time_since_start = time.time() - self.start_time

        while self.glob_coord_poller.poll(0.):
            data = pickle.loads(self.glob_coord_socket.recv())
            self.glob_coords = data[0]
            self.heading = data[1]

        # * big orange cone position estimation
        if self.keep_straight:
            if np.count_nonzero(world_state[:, 2] == CONE_CLASSES["big"]) > 0:
                self.process_big_cones(world_state.copy())
                self.estimate_map_position()
            else:
                self.estimate_map_position()
                self.current_path = self.circles[0]
                self.keep_straight = False

        if self.keep_straight or self.finish_detect:
            world_state = world_state[world_state[:, 2] == CONE_CLASSES["big"]] if self.keep_straight else world_state[world_state[:, 2] == CONE_CLASSES["orange"]]
            path = get_orange_centerline(world_state)
            delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

            if self.finish_detect:
                world_state[:, 2] = CONE_CLASSES["big"]
                dist_to_finish = get_big_orange_distance(cone_preds=world_state, min_big_cones=4)

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
            path = self.global_to_local(self.current_path.copy())[:5, :]
            delta, controller_log = stanley_steering(path, self.lookahead_dist, wheel_speed, self.linear_gain, self.nonlinear_gain)

        debug_dict = {
            "Glob coords": self.glob_coords,
            "Time since start": time_since_start,
            "Finish time": self.finished_time
            # "circle_centers": self.circle_centers,
            # "cone_centers": self.estimated_cone_centers
        }

        return self.finished, delta, self.speed_set_point, debug_dict, (path, controller_log["target"])

    def check_passed_through_center(self):
        map_center = np.sum(self.estimated_cone_centers, axis=0)/2
        if np.linalg.norm(self.glob_coords - map_center) <= Skidpad.WAYPOINT_RADIUS_SQ:
            if self.center_pass_counter == 2:
                self.current_path = self.circles.pop()
            elif self.center_pass_counter == 4:
                self.finish_detect = True
            self.center_pass_counter += 1
            self.last_passed_center = time.time()

    def global_to_local(self, path):
        """
        Works now
        """
        car_heading = np.deg2rad(self.heading)
        R = np.array([[np.cos(car_heading), -np.sin(car_heading)],
                      [np.sin(car_heading), np.cos(car_heading)]])
        path -= self.glob_coords
        path = (R.T @ path.T).T
        dists = np.sqrt(path[:, 0]**2 + path[:, 1]**2)
        path = path[np.argsort(dists, axis=0)]
        return path[path[:, 0] > 0]

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
        coords1 = np.array([np.array([np.cos(pt)*d, np.sin(pt)*d]) + c1 for pt in np.linspace(0, 2*np.pi, num=50)])
        coords2 = np.array([np.array([np.cos(pt)*d, np.sin(pt)*d]) + c2 for pt in np.linspace(0, 2*np.pi, num=50)])
        self.circle_centers = [c1, c2]
        self.circles = [coords1, coords2]
