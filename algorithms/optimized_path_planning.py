import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

from scipy.optimize import minimize
from scipy.interpolate import splprep, splev

class PathPlanning():
    def __init__(self, n_steps=None, debug=False):
        self.used_blue_cones = np.reshape([], (-1,2))  # only for debugging purpose?
        self.used_yellow_cones = np.reshape([], (-1,2)) # only for debugging purpose?

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

        self.log = {}
        self.log["args"] = {
            "debug" : self.debug,
            "n steps" : self.n_steps,
            "filling cone distance" : self.FILLING_CONE_DIST,
            "max path length" : self.MAX_PATH_LENGTH,
            "new_center_angle_limit" : self.NEW_CENTER_ANGLE_LIMIT
            }
        self.log["planned path"] = self.path

        self.time_records = {
            "Fill missing" : 0.,
            "Recolor orange cones": 0.,
            "Sort cones" : 0.,
            "Filter cones" : 0.,
            "Compute hats" : 0.,
            "Compute new center": 0.,
            "Check center location": 0.
        }
    
    def reset(self):
        # reset some variables for the next find_path() call
        self.used_blue_cones = np.reshape([], (-1,2))  # only for debugging purpose?
        self.used_yellow_cones = np.reshape([], (-1,2)) # only for debugging purpose?
        self.path = np.array([[0., 0.]])
        self.path_length = 0.
        self.step = 0

        self.time_records = {
            "Fill missing" : 0.,
            "Recolor orange cones": 0.,
            "Sort cones" : 0.,
            "Filter cones" : 0.,
            "Compute hats" : 0.,
            "Compute new center": 0.,
            "Check center location": 0.
        }
    
    def get_path_planning_log(self) -> dict:
        return self.log
    
    def get_time_records(self):
        return self.time_records
    
    def sort_cones(self, cones: np.array) -> np.array:
        # Sort cones in a right sequence: 
        # firstly find the closest cone to (0,0) and add it to sorted cones
        # then find the closest cone to the last sorted cone and add it.
        sorted_cones = np.empty(shape=cones.shape)     
        start = self.path[0]
        for i in range (len(cones)):
            # sort cones according to the current start point
            closest_cone, closest_cone_idx = closest_point(cones, start) 
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
        distances[0] -= self.MAX_DIST_TO_FIRST_CONE + self.MAX_DIST_BETWEEN_CONES

        # stop until one distance is larger than permitted
        # thelarge distance can mean that the next part of track is far away or parallel
        # slice_until = np.argmax(distances > self.MAX_DIST_BETWEEN_CONES)
        larger_distance = np.where(distances > self.MAX_DIST_BETWEEN_CONES)[0]
        slice_until = larger_distance[0] if len(larger_distance) > 0 else len(cones)

        return cones[0:slice_until]

    def fill_missing_cones(self, B: np.array, Y: np.array):
        # Fill cones by Matej Zorek. Input is SORTED blue and yellow cones. 
        # Usually fill missing cones in a turn (sometimes the car doesn't see inner cones in a turn)
        # Parallelogram method of adding vector is applied in this function. 
        B, Y = self.sort_cones(B), self.sort_cones(Y)
        full_yellow = True if len(Y) > len(B) else False
        full = Y if full_yellow else B
        to_fill = np.reshape([], (-1,2))
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
                    vec = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ vec2
                else:
                    vec = np.array([[np.cos(-angle), -np.sin(-angle)], [np.sin(-angle), np.cos(-angle)]]) @ vec2

            to_fill = np.vstack((to_fill, full[idx, :] + vec * self.FILLING_CONE_DIST))

        self.log[self.str_step]["fill missing cones"] = True
        if full_yellow:
            self.log[self.str_step]["filled blue cones"] = to_fill
            return to_fill, full
        else:
            self.log[self.str_step]["filled yellow cones"] = to_fill
            return full, to_fill
    
    def recolor_orange_cones(self, B_cones: np.array, Y_cones: np.array, O_cones: np.array):
        # Use orange cones to plan a path. 
        # Means to recolor/assign orange cones to blue/yellow and after it will be used in usual procedure.
        distances = [] # [:, 0] is yellow; [:, 1] is blue
        recolored_to_blue = np.reshape([], (-1,2))
        recolored_to_yellow = np.reshape([], (-1,2))

        for o_cone in O_cones:
            distances.append([
                np.linalg.norm(closest_point(Y_cones, o_cone)[0] - o_cone),
                np.linalg.norm(closest_point(B_cones, o_cone)[0] - o_cone)
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
        
        self.log[self.str_step]["recolor orange cones"] = True
        self.log[self.str_step]["recolored to blue"] = recolored_to_blue
        self.log[self.str_step]["recolored to yellow"] = recolored_to_yellow
        
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
        self.log[self.str_step]["b"], self.log[self.str_step]["y"] = b, y

        return b, y
    
    def calculate_center(self, pointB, pointY):
        center = (pointB-pointY) / 2 + pointY
        self.log[self.str_step]["center"] = center
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
            self.log[self.str_step]["new_center_angles"] = (angle_1, angle_2)

            if angle_1 > self.NEW_CENTER_ANGLE_LIMIT or angle_2 > self.NEW_CENTER_ANGLE_LIMIT:
                return False  # angle isn't ok
            else:
                return True
            
        new_center_is_ok = True
        dist_to_new_center = np.linalg.norm(new_center - self.path[-1])
        # check path length
        if self.path_length + dist_to_new_center > self.MAX_PATH_LENGTH:
            self.log[self.str_step]["info"] = "OK: path planning ended (path is far enough long)"
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
            self.log[self.str_step]["info"] = "NOT OK: path planning ended (new center angle is not ok)"
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
                k = -1 / k # k of normal line
                c = point2[1] - k * point2[0]
                self.log[self.str_step]["normal_k_coef"] = k
                self.log[self.str_step]["normal_c_coef"] = c
                return points[np.sign(points[:, 1] - k * points[:, 0] - c) == np.sign(-k)]
            else:
                #?shouldn't happen
                # TEST this case!!!
                c = point1[0]
                self.log[self.str_step]["normal_k_coef"] = 0
                self.log[self.str_step]["normal_c_coef"] = 0
                return points[np.sign(points[:, 0] - c) == np.sign(1)]
        B_hat = points_above_normal_line(B)
        Y_hat = points_above_normal_line(Y)
        
        return B_hat, Y_hat
    
    def find_path(self, B_cones, Y_cones, O_cones = []) -> np.array:
        """
        The main function of the class.
        Compute a center line path using all cones (Blue, Yellow, Big Orange)

        :param cones: cones from perception

        :return path: computed path
        """
        # logging
        self.str_step = f"step {self.step}"
        self.log[self.str_step] = {}
        self.log[self.str_step]["blue cones"] = B_cones
        self.log[self.str_step]["yellow cones"] = Y_cones
        self.log[self.str_step]["big orange cones"] = O_cones
        self.log[self.str_step]["fill missing cones"] = False

        # return if n_steps is negative
        if self.n_steps is not None and self.n_steps < 0:
            self.log[self.str_step]["info"] = "ERROR: n_steps < 0"
            return self.path
        
        if self.n_steps is not None and self.n_steps == self.step:
            self.log[self.str_step]["info"] = "OK: path planning is ended (n_steps==step)"
            return self.path

        # plot_path(self.path, B_cones, Y_cones, BIG_ORANGE_cones=[], b_bound=B_cones, y_bound=Y_cones)

        # If there are no Blue or Yellow cones, fill missing Blue or Yellow cones
        if len(B_cones) == 0 or len(Y_cones) == 0:
            if len(B_cones) == 0 and len(Y_cones) == 0:
                self.log[self.str_step]["info"] = "ERROR: not enough Blue/Yellow cones at the start"
                return self.path   
            else:
                start = time.time()
                B_cones, Y_cones = self.fill_missing_cones(B_cones, Y_cones)
                end = time.time()
                self.time_records["Fill missing"] += round((end-start)*1000, 4)
                

        # Recolor/assign big orange cones
        if len(O_cones) > 0:
            start = time.time()
            B_cones, Y_cones = self.recolor_orange_cones(B_cones, Y_cones, O_cones)
            end = time.time()
            self.time_records["Recolor orange cones"] += round((end-start)*1000, 4)

        # Sort cones
        start = time.time()
        B_cones, Y_cones = self.sort_cones(B_cones), self.sort_cones(Y_cones)
        end = time.time()
        self.time_records["Sort cones"] += round((end-start)*1000, 4)
        
        self.log[self.str_step]["sorted B"] = B_cones
        self.log[self.str_step]["sorted Y"] = Y_cones

        # Filter cones
        start = time.time()
        B, Y = self.filter_cones(B_cones), self.filter_cones(Y_cones)
        end = time.time()
        self.time_records["Filter cones"] += round((end-start)*1000, 4)
        
        # in log filtered B/Y is the same as B/Y,
        self.log[self.str_step]["B"], self.log[self.str_step]["Y"] = B, Y

        # the first cone was so far away
        if len(B) == 0 or len(Y) == 0:
            self.log[self.str_step]["info"] = "ERROR: not enough Blue/Yellow cones after filtering cones"
            return self.path
        
        B_hat, Y_hat = B, Y
        while len(B_hat) > 0 and len(Y_hat) > 0:
            self.log[self.str_step]["B hat"], self.log[self.str_step]["Y hat"] = B_hat, Y_hat

            start = time.time()
            # Find the closest b, y cones from B hat, Y hat to the last point of Path
            b, y = self.find_blue_and_yellow_cone(B_hat, Y_hat)
            # Calculate center between b and y
            center = self.calculate_center(b, y)
            end = time.time()
            self.time_records["Compute new center"] += round((end-start)*1000, 4)
            
            # Check correctness of the center placement
            start = time.time()
            new_center_is_ok = self.check_new_center_placement(center)
            end = time.time()
            self.time_records["Check center location"] += round((end-start)*1000, 4)
            if not new_center_is_ok:
                # doesn't pass distance or angle check
                break

            # Add center to Path
            self.path = np.vstack((self.path, center))

            # Compute next B_hat, Y_hat
            start = time.time()
            B_hat, Y_hat = self.compute_hats(B, Y)
            end = time.time()
            self.time_records["Compute hats"] += round((end-start)*1000, 4)
            
            
            self.log[self.str_step]["path"] = self.path
            self.log[self.str_step]["path length"] = self.path_length
            self.log[self.str_step]["done"] = True
            self.step += 1
        
        if not "info" in self.log[self.str_step]:
            self.log[self.str_step]["info"] = "OK: path planning is ended (B_hat or Y_hat is empty)"

        self.log["planned path"] = self.path
        return self.path

# According to Stanley paper
def stanley_smooth_path(path, use_spline_as_smoother=False):

    def normalize(v):
        norm = np.linalg.norm(v,axis=0) + 0.00001
        return v / norm.reshape(1, v.shape[1])

    def curvature(waypoints):
        '''
        Curvature as  the sum of the normalized dot product between the way elements
        Implement second term of the smoothing objective.

        args: 
            waypoints [2, num_waypoints] !!!!!
        '''
        shift_left  = np.roll(waypoints, shift=-1, axis=1)
        shift_right = np.roll(waypoints, shift=1,  axis=1)

        left_half   = normalize(shift_left - waypoints)
        right_half  = normalize(waypoints  - shift_right)

        segment     = np.array([np.dot(l, r) for l, r in zip(left_half.T, right_half.T)])

        segment[0]  = 0 # right half of this connects first to last waypoint, delete
        segment[-1] = 0 # left  half of this connects first to last waypoint, delete

        return np.sum(segment)
    
    def smoothing_objective(waypoints, waypoints_center, weight_curvature=16): #weight_curvature=128
        '''
        Objective for path smoothing

        args:
            waypoints [2 * num_waypoints] !!!!!
            waypoints_center [2 * num_waypoints] !!!!!
            weight_curvature (default=40)
        '''
        waypoints = waypoints.reshape(-1, 2)
        waypoints_center = waypoints_center.reshape(-1, 2)

        # time perfomance: (one step/iteration?; so many steps)
        # ls to center took: 0.0412 ms
        # ls curveture took: 0.433 ms

        # mean least square error between waypoint and way point center
        start = time.time()
        ls_tocenter = np.sum(np.abs(waypoints_center - waypoints)**2)
        end = time.time()
        # print(f"ls to center took: {round((end-start)*1000, 4)} ms")

        # derive curvature
        start = time.time()
        ls_curvature = curvature(waypoints.T)
        end = time.time()
        # print(f"ls curveture took: {round((end-start)*1000, 4)} ms")
        # print("---")
        

        boundary_penalty = 0 # ? TODO? F_rddf in Stanley paper     

        return ls_tocenter - weight_curvature * ls_curvature + boundary_penalty

    # increase amount of points on the path
    # (first step in stanley alg)


    # TODO initialize to previous path previous path -> (path)

    constraints_dict = (
        {"type": 'ineq', "fun": lambda x: x[0] - path[0]},
        {"type": 'ineq', "fun": lambda x: x[1] - path[1]},
        {"type": 'ineq', "fun": lambda x: -x[0] + path[0]},
        {"type": 'ineq', "fun": lambda x: -x[1] + path[1]}
    )

    opts = {"maxiter": 3, "disp": False}
    # opts = {"maxiter": 3, "disp": True}

    # constraints_dict = (
    #     {"type": 'eq', "fun": lambda x: x[0] - path[0]},
    #     {"type": 'eq', "fun": lambda x: x[1] - path[1]}
    # )

    add_more_points_to_path = False

    if use_spline_as_smoother:
        add_more_points_to_path = True

    if add_more_points_to_path:
        if len(path) > 3:
            start = time.time()
            spline_smoothness = 10
            spl_path, _ = splprep((path[:,0], path[:, 1]), s=spline_smoothness)
            # create spline arguments
            num_waypoints = 5 #8 #13 # 15
            t = np.linspace(0, 1, num_waypoints)
            # derive roadside points from spline
            new_path = np.array(splev(t, spl_path)).T
            end = time.time()
            # print(f"Add more point to path took: {round((end-start)*1000, 4)} ms")
        else:
            new_path = path

        start = time.time()
        if use_spline_as_smoother:
            return new_path
        else:
            way_points = minimize(smoothing_objective, (new_path), args=new_path, options=opts)

        end = time.time()
        # print(f"Minimize took: {round((end-start)*1000, 4)} ms")
        # print()
    else:
        start = time.time()
        way_points = minimize(smoothing_objective, (path), args=path, options=opts)
        # way_points = minimize(smoothing_objective, (path), args=path, constraints=constraints_dict) # so slow
        # way_points = minimize(smoothing_objective, (path), args=path)

        end = time.time()
        # print(f"Minimize took: {round((end-start)*1000, 4)} ms")
        # print()
       
        
    # way_points = minimize(smoothing_objective, (path), args=path)
    
    way_points = way_points["x"]

    smooth_path = way_points.reshape(-1, 2)
    smooth_path[0] = [0., 0.] # TODO:Fix this ?add constraint to minimize?

    return smooth_path

                 
def closest_point(arr, point):
    dist = (arr[:, 0] - point[0])**2 + (arr[:, 1] - point[1])**2
    idx = dist.argmin()
    return (arr[idx], idx)

def plot_path(path, B_cones, Y_cones, BIG_ORANGE_cones=[], b_bound=[], y_bound=[]):
        # plot a path with blue and yellow cones
        fig = plt.figure(1, figsize=(20, 10))
        axe = fig.add_axes([0.05, 0.05, 0.9, 0.9])
        axe.set_aspect("equal")

        if len(B_cones) > 0:
            axe.plot(B_cones[:, 0], B_cones[:, 1], "o", color="blue", mec ="black")
        if len(Y_cones) > 0:
            axe.plot(Y_cones[:, 0], Y_cones[:, 1], "o", color="yellow", mec ="black")
        if len(BIG_ORANGE_cones) > 0:
            axe.plot(BIG_ORANGE_cones[:, 0], BIG_ORANGE_cones[:, 1], "o", color="orange", mec ="black")
        
        if len(b_bound) > 0:
            axe.plot(b_bound[:, 0], b_bound[:, 1], color="blue")
        if len(y_bound) > 0:
            axe.plot(y_bound[:, 0], y_bound[:, 1], color="yellow")

        axe.plot(path[:, 0], path[:, 1], color="red")
        axe.plot(path[:, 0], path[:, 1], "o", color="red", mec ="black")

        plt.show()


class PathPlanner():
    def __init__(self, n_steps=None, debug=False):
        self.n_steps = n_steps
        self.debug = debug
        self.planner = PathPlanning(n_steps=n_steps)

        self.path_planning_time_records = pd.DataFrame(data = {
            "Fill missing" : [],
            "Recolor orange cones": [],
            "Sort cones" : [],
            "Filter cones" : [],
            "Compute hats" : [],
            "Compute new center": [],
            "Check center location": [],
            "Path Planning": []
            # "Path Planning": [],
            # "Path Smoothing": [],
            # "Path Planning and Smoothing": []
        })

    
    def find_path(self, args):
        # args in the past: blue cones, yellow cones
        # args now: cones; where cones contain blue, yellow and orange cones

        # if len(args) == 1:
        #     cones = args[0]
        #     yellow_cones = cones[cones[:, 2] == 0, :2]
        #     blue_cones = cones[cones[:, 2] == 1, :2]
        #     orange_cones = cones[cones[:, 2] == 3, :2]
        # elif len(args) == 2:
        #     blue_cones, yellow_cones = args
        #     orange_cones = []
        # else:
        #     raise ValueError("Len (args) > 2")
        
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
            start_path_planning_time = time.time()
            path = self.planner.find_path(blue_cones, yellow_cones, O_cones=orange_cones)
            end_path_planning_time = time.time()
            self.path_planning_time = round((end_path_planning_time-start_path_planning_time)*1000, 2)
            # print(f"Path planning took: {self.path_planning_time} ms")

            # start_path_smoothing_time = time.time()
            # path = self.planner.stanley_smooth_path(path)
            # end_path_smoothing_time = time.time()
            # self.path_smoothing_time = round((end_path_smoothing_time-start_path_smoothing_time)*1000, 2)

            # print(f"Path smoothing took: {self.path_smoothing_time} ms")
            # print(f"Path planning and smoothing took: {self.path_planning_time+self.path_smoothing_time} ms")
        except:
            path = np.array([[0., 0.]])

        pp_time_records = self.set_path_planning_time_records()
        self.path_planning_time_records.loc[len(self.path_planning_time_records.index)] = list(pp_time_records.values())
        return path
    
    def set_path_planning_time_records(self):
        time_records = self.planner.get_time_records()
        # time_records={}
        time_records["Path Planning"] = self.path_planning_time
        # time_records["Path Smoothing"] = self.path_smoothing_time
        # time_records["Path Planning and Smoothing"] = self.path_planning_time+self.path_smoothing_time

        return time_records

    def get_path_planning_time_records(self):
        return self.path_planning_time_records