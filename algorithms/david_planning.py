import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

from scipy.optimize import minimize
from scipy.interpolate import splprep, splev
import torch
from torch import optim


class PathPlanning():
    def __init__(self, n_steps=None, debug=False):
        self.used_blue_cones = np.reshape(
            [], (-1, 2))  # only for debugging purpose?
        self.used_yellow_cones = np.reshape(
            [], (-1, 2))  # only for debugging purpose?
        self.FILLING_CONE_DIST = 3.5        # is used to fill missing cones
        self.MAX_PATH_LENGTH = 20.          # can be used to stop path planning
        self.MAX_DIST_TO_FIRST_CONE = 10.   # is used in filtering cones #? delete
        self.MAX_DIST_BETWEEN_CONES = 6.    # is used in filtering cones
        self.NEW_CENTER_ANGLE_LIMIT = 250.  # 235.  # in degrees
        self.n_steps = n_steps
        self.debug = debug
        self.path = np.array([[0., 0.]])
        self.path_length = 0.
        self.step = 0

    def reset(self):
        # reset some variables for the next find_path() call
        self.used_blue_cones = np.reshape(
            [], (-1, 2))  # only for debugging purpose?
        self.used_yellow_cones = np.reshape(
            [], (-1, 2))  # only for debugging purpose?
        self.path = np.array([[0., 0.]])
        self.path_length = 0.
        self.step = 0

    def sort_cones(self, cones: np.array) -> np.array:
        # Sort cones in a right sequence:
        # firstly find the closest cone to (0,0) and add it to sorted cones
        # then find the closest cone to the last sorted cone and add it.
        sorted_cones = np.empty(shape=cones.shape)
        start = self.path[0]
        for i in range(len(cones)):
            # sort cones according to the current start point
            closest_cone, closest_cone_idx = self.closest_point(cones, start)
            cones = np.delete(cones, closest_cone_idx, axis=0)
            sorted_cones[i] = closest_cone
            start = closest_cone

        return sorted_cones

    def filter_cones(self, cones):
        # Filter cones accordnig to FS rules:
        # - the distance between blue or yellow cones is up to 6m

        # ? delete cones that are behind (0,0) <-> x<0, and y<0

        # add the origin to the cones and shift them by one, then calculate the norm between elements
        shifted_cones = np.vstack((np.array([0., 0.]), cones))[0:-1]
        distances = np.linalg.norm(cones - shifted_cones, axis=1)
        # compensate for the larger permitted distance to the first cone
        distances[0] -= self.MAX_DIST_TO_FIRST_CONE + \
            self.MAX_DIST_BETWEEN_CONES

        # stop until one distance is larger than permitted
        # thelarge distance can mean that the next part of track is far away or parallel
        # slice_until = np.argmax(distances > self.MAX_DIST_BETWEEN_CONES)
        larger_distance = np.where(distances > self.MAX_DIST_BETWEEN_CONES)[0]
        slice_until = larger_distance[0] if len(
            larger_distance) > 0 else len(cones)

        return cones[0:slice_until]

    def fill_missing_cones(self, B: np.array, Y: np.array):
        # Fill cones by Matej Zorek. Input is SORTED blue and yellow cones.
        # Usually fill missing cones in a turn (sometimes the car doesn't see inner cones in a turn)
        # Parallelogram method of adding vector is applied in this function.
        B, Y = self.sort_cones(B), self.sort_cones(Y)
        full_yellow = True if len(Y) > len(B) else False
        full = Y if full_yellow else B
        to_fill = np.reshape([], (-1, 2))
        matrix_rotation_90_deg = np.array([[0, -1], [1, 0]])
        matrix_rotation_270_deg = np.array([[0, 1], [-1, 0]])

        for idx in range(len(full)):
            if idx == 0:
                vec = full[1, :] - full[0, :]
                vec = vec / np.linalg.norm(vec)
                vec = matrix_rotation_90_deg @ vec if full_yellow else matrix_rotation_270_deg @ vec
            elif idx == len(full) - 1:
                vec = full[-1, :] - full[-2, :]
                vec = vec / np.linalg.norm(vec)
                vec = matrix_rotation_90_deg @ vec if full_yellow else matrix_rotation_270_deg @ vec
            else:
                vec1 = full[idx - 1, :] - full[idx, :]
                vec2 = full[idx + 1, :] - full[idx, :]
                vec1 = vec1 / np.linalg.norm(vec1)
                vec2 = vec2 / np.linalg.norm(vec2)
                angle = np.arccos(np.dot(vec1, vec2)) / 2

                if full_yellow:
                    vec = np.array(
                        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ vec2
                else:
                    vec = np.array(
                        [[np.cos(-angle), -np.sin(-angle)], [np.sin(-angle), np.cos(-angle)]]) @ vec2

            to_fill = np.vstack(
                (to_fill, full[idx, :] + vec * self.FILLING_CONE_DIST))

        if full_yellow:
            return to_fill, full
        else:
            return full, to_fill

    def closest_point(self, arr, point):
        dist = (arr[:, 0] - point[0])**2 + (arr[:, 1] - point[1])**2
        idx = dist.argmin()
        return (arr[idx], idx)

    def recolor_orange_cones(self, B_cones: np.array, Y_cones: np.array, O_cones: np.array):
        # Use orange cones to plan a path.
        # Means to recolor/assign orange cones to blue/yellow and after it will be used in usual procedure.
        distances = []  # [:, 0] is yellow; [:, 1] is blue
        recolored_to_blue = np.reshape([], (-1, 2))
        recolored_to_yellow = np.reshape([], (-1, 2))

        for o_cone in O_cones:
            distances.append([
                np.linalg.norm(self.closest_point(Y_cones, o_cone)[0] - o_cone),
                np.linalg.norm(self.closest_point(B_cones, o_cone)[0] - o_cone)
            ])

        my_sort_dist = sorted(distances, key=lambda x: min(x[0], x[1]))
        b_count = 0
        y_count = 0
        for dist in my_sort_dist:
            if dist[0] < dist[1]:
                if y_count == 2:
                    # recolor to blue because yellows are enough
                    recolored_to_blue = np.vstack(
                        (recolored_to_blue, O_cones[distances.index(dist)]))
                    b_count += 1
                    continue

                # recolor to yellow
                recolored_to_yellow = np.vstack(
                    (recolored_to_yellow, O_cones[distances.index(dist)]))
                y_count += 1
            else:
                if b_count == 2:
                    # recolor to yellow because blues are enough
                    recolored_to_yellow = np.vstack(
                        (recolored_to_yellow, O_cones[distances.index(dist)]))
                    y_count += 1
                    continue

                # recolor to blue
                recolored_to_blue = np.vstack(
                    (recolored_to_blue, O_cones[distances.index(dist)]))
                b_count += 1

        B = np.vstack((B_cones, recolored_to_blue))
        Y = np.vstack((Y_cones, recolored_to_yellow))

        return B, Y

    def find_blue_and_yellow_cone(self, B_hat, Y_hat):
        # take the closest blue and yellow cones to the last point of path
        # b, b_idx = closest_point(B_hat, self.path[-1])
        # y, y_idx = closest_point(Y_hat, self.path[-1])

        # B_hat and Y_hat are already sorted
        b = B_hat[0]
        y = Y_hat[0]
        return b, y

    def calculate_center(self, pointB, pointY):
        center = (pointB - pointY) / 2 + pointY
        return center

    def check_new_center_placement(self, new_center):
        def check_angle_between_vectors(vector_1, vector_2):
            # compute angle in degrees
            if all(vector_1 == [0, 0]) or all(vector_2 == [0, 0]):
                return 0

            unit_vector1 = vector_1 / np.linalg.norm(vector_1)
            unit_vector2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector1, unit_vector2)

            if dot_product < -1.0:
                dot_product = -1.0
            elif dot_product > 1.0:
                dot_product = 1.0

            angle_1 = np.degrees(np.arccos(dot_product))
            angle_2 = 360 - angle_1

            if angle_1 > self.NEW_CENTER_ANGLE_LIMIT or angle_2 > self.NEW_CENTER_ANGLE_LIMIT:
                return False  # angle isn't ok
            else:
                return True

        new_center_is_ok = True
        dist_to_new_center = np.linalg.norm(new_center - self.path[-1])
        # check path length
        if self.path_length + dist_to_new_center > self.MAX_PATH_LENGTH:
            return False

        if len(self.path) < 2:
            # not enough points in path to compute angle
            self.path_length += dist_to_new_center
            return new_center_is_ok

        # check angle between path[-1] and new center
        vector_1 = self.path[-2] - self.path[-1]
        vector_2 = new_center - self.path[-1]

        if not check_angle_between_vectors(vector_1, vector_2):
            # doesn't pass angle check
            return False

        # new center is ok
        self.path_length += dist_to_new_center
        return new_center_is_ok

    def compute_hats(self, B: np.array, Y: np.array):
        # Compute B hat and Y hat.
        # Hat means what is above the normal line to path[-1]-path[-2] in path[-1].

        def points_above_normal_line(points):
            point1, point2 = self.path[-2], self.path[-1]
            if point2[0] - point1[0] != 0:
                k = (point2[1] - point1[1]) / (point2[0] - point1[0])
                k = -1 / k  # k of normal line
                c = point2[1] - k * point2[0]
                return points[np.sign(points[:, 1] - k * points[:, 0] - c) == np.sign(-k)]
            else:
                # ?shouldn't happen
                # TEST this case!!!
                c = point1[0]
                return points[np.sign(points[:, 0] - c) == np.sign(1)]
        B_hat = points_above_normal_line(B)
        Y_hat = points_above_normal_line(Y)

        return B_hat, Y_hat

    def find_path(self, B_cones, Y_cones, O_cones=[]) -> np.array:
        """
        The main function of the class.
        Compute a center line path using all cones (Blue, Yellow, Big Orange)

        :param cones: cones from perception

        :return path: computed path
        """
        # return if n_steps is negative
        if self.n_steps is not None and self.n_steps < 0:
            return self.path

        if self.n_steps is not None and self.n_steps == self.step:
            return self.path

        # plot_path(self.path, B_cones, Y_cones, BIG_ORANGE_cones=[], b_bound=B_cones, y_bound=Y_cones)

        # If there are no Blue or Yellow cones, fill missing Blue or Yellow cones
        if len(B_cones) == 0 or len(Y_cones) == 0:
            if len(B_cones) == 0 and len(Y_cones) == 0:
                return self.path
            else:
                B_cones, Y_cones = self.fill_missing_cones(B_cones, Y_cones)

        # Recolor/assign big orange cones
        if len(O_cones) > 0:
            B_cones, Y_cones = self.recolor_orange_cones(
                B_cones, Y_cones, O_cones)

        # Sort cones
        B_cones, Y_cones = self.sort_cones(B_cones), self.sort_cones(Y_cones)
        # Filter cones
        B, Y = self.filter_cones(B_cones), self.filter_cones(Y_cones)

        # in log filtered B/Y is the same as B/Y,

        # the first cone was so far away
        if len(B) == 0 or len(Y) == 0:
            return self.path

        B_hat, Y_hat = B, Y
        while len(B_hat) > 0 and len(Y_hat) > 0:
            # start = time.time()
            # Find the closest b, y cones from B hat, Y hat to the last point of Path
            b, y = self.find_blue_and_yellow_cone(B_hat, Y_hat)
            # Calculate center between b and y
            center = self.calculate_center(b, y)
            # end = time.time()
            # self.time_records["Compute new center"] += round((end - start) * 1000, 4)

            # Check correctness of the center placement
            # start = time.time()
            new_center_is_ok = self.check_new_center_placement(center)
            # self.time_records["Check center location"] += round((end - start) * 1000, 4)
            if not new_center_is_ok:
                # doesn't pass distance or angle check
                break

            # Add center to Path
            self.path = np.vstack((self.path, center))

            # Compute next B_hat, Y_hat
            # start = time.time()
            B_hat, Y_hat = self.compute_hats(B, Y)
            # end = time.time()
            # self.time_records["Compute hats"] += round((end - start) * 1000, 4)
        return self.path


class PathPlanner():
    def __init__(self, opt: dict = None):
        if opt is None:
            self.n_steps = 8
        else:
            self.n_steps = opt["n_steps"]
        self.planner = PathPlanning(n_steps=self.n_steps)

    def find_path(self, args):
        # args in the past: blue cones, yellow cones
        # args now: cones; where cones contain blue, yellow and orange cones
        cones = args
        if cones is None:
            blue_cones = np.zeros((0, 3))
            yellow_cones = np.zeros((0, 3))
        else:
            yellow_cones = cones[cones[:, 2] == 0, :2]
            blue_cones = cones[cones[:, 2] == 1, :2]
            orange_cones = cones[cones[:, 2] == 3, :2]
        try:
            self.planner.reset()
            path = self.planner.find_path(
                blue_cones, yellow_cones, O_cones=orange_cones)
        except Exception as e:
            print(e)
            path = np.array([[0., 0.]])
        return path
