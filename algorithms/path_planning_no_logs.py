import numpy as np


class PathPlanning:

    def __init__(self, debug=False, n_steps=None):

        self.used_blue_cones = np.reshape([], (-1, 2))
        self.used_yellow_cones = np.reshape([], (-1, 2))
        self.path = np.array([[0., 0.]])

        self.FILLING_CONE_DIST = 3.5        # is used to fill missing cones
        self.MAX_PATH_LENGTH = 20.          # can be used to stop path planning
        self.MAX_DIST_TO_FIRST_CONE = 10.   # is used in filtering cones
        self.MAX_DIST_BETWEEN_CONES = 6.    # is used in filtering cones

        self.n_steps = n_steps

        self.path_length = 0.
        self.debug = debug

    def reset(self):
        # reset some variables for the next find_path() call
        self.used_blue_cones = np.reshape(
            [], (-1, 2))  # only for debugging purpose?
        self.used_yellow_cones = np.reshape(
            [], (-1, 2))  # only for debugging purpose?
        self.path = np.array([[0., 0.]])
        self.path_length = 0.
        self.step = 0

    def is_already_added(self, point: np.array, among_points: np.array) -> bool:
        return np.all(np.isin(point, among_points))

    def is_new_center_ok(self, new_center):
        def compute_angle_between_vectors(vector_1, vector_2):
            if all(vector_1 == [0, 0]) or all(vector_2 == [0, 0]):
                return 0

            unit_vector1 = vector_1 / np.linalg.norm(vector_1)
            unit_vector2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector1, unit_vector2)

            if dot_product < -1.0:
                dot_product = -1.
            elif dot_product > 1.0:
                dot_product = 1.

            angle = np.degrees(np.arccos(dot_product))
            return angle

        vector_1 = np.array([self.path[-2][0] - self.path[-1][0],
                             self.path[-2][1] - self.path[-1][1]])
        vector_2 = np.array([new_center[0] - self.path[-1][0],
                             new_center[1] - self.path[-1][1]])

        angle_1 = compute_angle_between_vectors(vector_1, vector_2)
        angle_2 = 360 - angle_1

        max_degree_limit = 250.  # 235.
        if angle_1 > max_degree_limit or angle_2 > max_degree_limit:
            return False  # angle isn't ok
        else:
            return True

    def points_above_cone_line(self, points: np.array) -> np.array:
        """
        The line is y=kx+c
        """

        points = np.array([p for p in points if (not self.is_already_added(p, self.used_blue_cones) and not self.is_already_added(p, self.used_yellow_cones))])  # filter used points/ do i need this?

        if len(points) == 0:
            return []

        pointB, pointY = self.used_blue_cones[-1], self.used_yellow_cones[-1]

        if pointB[0] - pointY[0] != 0:
            k = (pointB[1] - pointY[1]) / (pointB[0] - pointY[0])
            c = pointB[1] - k * pointB[0]
            return points[np.sign(points[:, 1] - k * points[:, 0] - c) == np.sign(pointY[0] - pointB[0])]
        else:
            c = pointB[0]
            return points[np.sign(points[:, 0] - c) == np.sign(1)]

    def points_above_normal_line(self, points):
        point1, point2 = self.path[-2], self.path[-1]

        if point2[0] - point1[0] != 0:
            k = (point2[1] - point1[1]) / (point2[0] - point1[0])
            k = -1 / k  # k of normal line
            c = point2[1] - k * point2[0]
            return points[np.sign(points[:, 1] - k * points[:, 0] - c) == np.sign(-k)]
        else:
            return np.array([])  # ?shouldn't happen

    def find_blue_and_yellow_cone(self, B_points, Y_points):
        # ? consider interpolating the end of the path if there are more cones of one color
        # TODO: redo the function

        sorted_B_points = self.sort_points(B_points, self.path[-1])  # is neccessary to sort here?
        sorted_Y_points = self.sort_points(Y_points, self.path[-1])  # test

        if self.step == 1:
            b = sorted_B_points[0]
            y = sorted_Y_points[0]
            return b, y

        dist_to_blue_cone = np.linalg.norm(sorted_B_points[0] - self.path[-1])
        dist_to_yellow_cone = np.linalg.norm(sorted_Y_points[0] - self.path[-1])

        # use code below to have more center points
        if dist_to_blue_cone < dist_to_yellow_cone:
            b = sorted_B_points[0]
            y = self.used_yellow_cones[-1]
            # new condition for last cones (connects with normal line)
            if self.is_already_added(b, self.used_blue_cones) and self.is_already_added(y, self.used_yellow_cones):
                b = self.used_blue_cones[-1]
                y = sorted_Y_points[0]
        else:
            b = self.used_blue_cones[-1]
            y = sorted_Y_points[0]
            # new condition for last cones (connects with normal line)
            if self.is_already_added(b, self.used_blue_cones) and self.is_already_added(y, self.used_yellow_cones):
                b = sorted_B_points[0]
                y = self.used_yellow_cones[-1]

        return b, y

    def sort_points(self, points: np.array, start: np.array) -> np.array:
        """ Sort points by distance """
        sorted_points = sorted(points, key=lambda x: np.linalg.norm(x - start))
        sorted_points = np.reshape(sorted_points, (points.shape))
        return sorted_points

    def sort_cones(self, cones: np.array) -> np.array:
        """
        Sort cones in a right sequence.

        :param cones: Blue/Yellow cones

        :return sorted cones: sorted Blue/Yellow cones
        """
        # * optimized with numpy
        sorted_cones = np.empty(shape=cones.shape)
        start = self.path[0]
        for i in range(cones.shape[0]):
            cones = self.sort_points(cones, start)  # sort cones according to the current start point
            closest_cone = cones[0]
            cones = np.delete(cones, 0, axis=0)
            sorted_cones[i] = closest_cone
            start = closest_cone
        return sorted_cones

    def filter_cones(self, cones: np.array) -> np.array:
        """ Filter cones accordnig to FS rules:
            - the distance between blue or yellow cones is up to 6m
        """

        # * optimized with numpy

        # add the origin to the cones and shift them by one, then calculate the norm between elements
        shifted_cones = np.vstack((np.array([0., 0.]), cones))[0:-1]
        distances = np.linalg.norm(cones - shifted_cones, axis=1)
        distances[0] -= self.MAX_DIST_TO_FIRST_CONE + self.MAX_DIST_BETWEEN_CONES  # compensate for the larger permitted distance to the first cone

        # old code
        # for i in range(len(distances)):
        #     if distances[i] <= self.MAX_DIST_BETWEEN_CONES:
        #         filtered_cones.append(cones[i])
        #     elif i == 0 and distances[i] <= self.MAX_DIST_TO_FIRST_CONE:
        #         filtered_cones.append(cones[0])
        #     else:
        #         break

        return cones[distances <= self.MAX_DIST_BETWEEN_CONES]  # np.array(filtered_cones)

    def compute_hats(self, B: np.array, Y: np.array):
        """
        Compute B hat and Y hat. Hat means what is above the line/normal of last used cones/last added center.

        :param B: Blue cones
        :param Y: Yellow cones

        :return B_hat, Y_hat: computed Blue and Yellow hats
        """
        if self.step == 1:
            B_hat, Y_hat = B, Y
        else:  # self.step > 1
            # the line goes through the last used blue and yellow cones

            B_hat = self.points_above_cone_line(B)
            Y_hat = self.points_above_cone_line(Y)

            # in case of there are no cones above cone line, try to find cones above the normal (normal to path)
            if len(B_hat) == 0:
                B_hat = self.points_above_normal_line(B)
            if len(Y_hat) == 0:
                Y_hat = self.points_above_normal_line(Y)

        return B_hat, Y_hat

    def recolor_orange_cones(self, B_cones: np.array, Y_cones: np.array, O_cones: np.array):
        """
        Use orange cones to plan a path. Means to recolor orange cones to blue/yellow and after it will be used in usual procedure.

        :param B_cones: Blue cones
        :param Y_cones: Yellow cones
        :param O_cones: Big Orange cones

        :return B,Y: extended Blue and Yellow cones
        """

        distances = []  # [:, 0] is yellow; [:, 1] is blue
        recolored_to_blue = np.reshape([], (-1, 2))
        recolored_to_yellow = np.reshape([], (-1, 2))

        for o_cone in O_cones:
            distances.append([
                np.linalg.norm(self.sort_points(Y_cones, o_cone)[0] - o_cone),
                np.linalg.norm(self.sort_points(B_cones, o_cone)[0] - o_cone)
            ])

        my_sort_dist = sorted(distances, key=lambda x: min(x[0], x[1]))
        b_count = 0
        y_count = 0
        for dist in my_sort_dist:
            if dist[0] < dist[1]:
                if y_count == 2:
                    # recolor to blue because yellows are enough
                    recolored_to_blue = np.vstack((recolored_to_blue, O_cones[distances.index(dist)]))
                    b_count += 1
                    continue

                # recolor to yellow
                recolored_to_yellow = np.vstack((recolored_to_yellow, O_cones[distances.index(dist)]))
                y_count += 1
            else:
                if b_count == 2:
                    # recolor to yellow because blues are enough
                    recolored_to_yellow = np.vstack((recolored_to_yellow, O_cones[distances.index(dist)]))
                    y_count += 1
                    continue

                # recolor to blue
                recolored_to_blue = np.vstack((recolored_to_blue, O_cones[distances.index(dist)]))
                b_count += 1

        B = np.vstack((B_cones, recolored_to_blue))
        Y = np.vstack((Y_cones, recolored_to_yellow))

        return B, Y

    def fill_missing_cones(self, B: np.array, Y: np.array):
        """
        Fill cones by Matej Zorek. 
        Usually fill missing cones in a turn (sometimes the car doesn't see inner cones in a turn)

        :param B: Blue cones
        :param Y: Yellow cones

        :return (to_fill/full, full/to_fill): extended and full cones
        """

        yellow = True if len(Y) > len(B) else False
        filled_cones = np.reshape([], (-1, 2))

        if yellow:
            Y = self.sort_cones(Y)
            to_fill = B
            full = Y
        else:
            B = self.sort_cones(B)
            to_fill = Y
            full = B

        if len(to_fill) == 0:
            to_fill = to_fill.reshape(-1, full.shape[1])

        if len(full) < 2:
            # not enough cones
            if yellow:
                return to_fill, full
            else:
                return full, to_fill

        for idx in range(len(to_fill), len(full)):
            if idx == 0:
                vec = full[1, :] - full[0, :]
                vec = vec / np.linalg.norm(vec)
                if yellow:
                    vec = np.matmul(np.array([[0, -1], [1, 0]]), vec)  # 90 deg rotation
                else:
                    vec = np.matmul(np.array([[0, 1], [-1, 0]]), vec)
            elif idx == len(full) - 1:
                vec = full[-1, :] - full[-2, :]
                vec = vec / np.linalg.norm(vec)
                if yellow:
                    vec = np.matmul(np.array([[0, -1], [1, 0]]), vec)
                else:
                    vec = np.matmul(np.array([[0, 1], [-1, 0]]), vec)
            else:
                vec1 = full[idx - 1, :] - full[idx, :]
                vec2 = full[idx + 1, :] - full[idx, :]
                vec1 = vec1 / np.linalg.norm(vec1)
                vec2 = vec2 / np.linalg.norm(vec2)
                angle = np.arccos(np.dot(vec1, vec2)) / 2

                if yellow:
                    vec = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ vec2
                else:
                    vec = np.array([[np.cos(-angle), -np.sin(-angle)], [np.sin(-angle), np.cos(-angle)]]) @ vec2

            to_fill = np.vstack((to_fill, full[idx, :] + vec * self.FILLING_CONE_DIST))

        if yellow:
            return to_fill, full
        else:
            return full, to_fill

    def find_path(self, B_cones, Y_cones, O_cones=[]) -> np.array:
        """
        Compute a path using all cones (Blue, Yellow, Big Orange)

        :param cones: cones from perception

        :return path: computed path
        """

        self.step = 0
        if self.n_steps is not None and self.n_steps < 0:
            return self.path
        # fill missing cones (it's case with a turn)
        if len(B_cones) == 0 or len(Y_cones) == 0:
            # ? if no cones are found, return empty path
            if len(B_cones) == 0 and len(Y_cones) == 0:
                return self.path
            else:
                B_cones, Y_cones = self.fill_missing_cones(B_cones, Y_cones)

        # use Orange cones to plan a path
        if len(O_cones) > 1:
            B_cones, Y_cones = self.recolor_orange_cones(B_cones, Y_cones, O_cones)

        B, Y = self.sort_cones(B_cones), self.sort_cones(Y_cones)
        B, Y = self.filter_cones(B), self.filter_cones(Y)

        if len(B) == 0 or len(Y) == 0:
            return self.path

        while True:
            if self.n_steps is not None and self.step == self.n_steps:
                return self.path

            self.step += 1

            B_hat, Y_hat = self.compute_hats(B, Y)
            if len(B_hat) == 0 or len(Y_hat) == 0:
                return self.path

            b, y = self.find_blue_and_yellow_cone(B_hat, Y_hat)
            self.used_blue_cones = np.vstack((self.used_blue_cones, b))
            self.used_yellow_cones = np.vstack((self.used_yellow_cones, y))
            center = (b - y) / 2 + y
            if self.is_already_added(center, self.path):
                return self.path

            dist_to_new_center = np.linalg.norm(center - self.path[-1])
            if self.path_length + dist_to_new_center > self.MAX_PATH_LENGTH:
                return self.path

            # TODO: asi to potrebuju?
            if len(self.path) > 2 and not self.is_new_center_ok(center):
                return self.path

            # add new center to a path
            self.path = np.vstack((self.path, center))
            self.path_length += dist_to_new_center


class PathPlanner():
    def __init__(self, n_steps=False, debug=False):
        self.n_steps = n_steps
        self.debug = debug
        self.planner = PathPlanning()

    def find_path(self, cones):
        self.planner.reset()
        if cones is None:
            blue_cones = np.zeros((0, 3))
            yellow_cones = np.zeros((0, 3))
        else:
            yellow_cones = cones[cones[:, 2] == 0, :2]
            blue_cones = cones[cones[:, 2] == 1, :2]
            orange_cones = cones[cones[:, 2] == 3, :2]

        try:
            path = self.planner.find_path(blue_cones, yellow_cones, O_cones=orange_cones)
            # path = stanley_smooth_path(path)
        except Exception as e:
            print(f"path_planning {type(e)} occured: {e}")
            path = np.array([[0., 0.]])

        return path
