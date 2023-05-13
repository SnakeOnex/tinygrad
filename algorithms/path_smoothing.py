import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

from scipy.optimize import minimize
import torch
from torch import optim


class PathSmoothing():
    def __init__(self):
        pass

    def torch_smooth(self, path):
        initial_trajectory = torch.tensor(
            path, dtype=torch.float32, requires_grad=True)
        initial_trajectory_gt = initial_trajectory.clone()
        optimizer = optim.ASGD([initial_trajectory], lr=0.005)
        num_iterations = 5
        best_cost = float('inf')
        best_trajectory = initial_trajectory
        for i in range(num_iterations):
            optimizer.zero_grad()
            cost = self.torch_smoothing_objective(
                initial_trajectory, initial_trajectory_gt)
            if cost.item() < best_cost:
                best_cost = cost.item()
                best_trajectory = initial_trajectory.clone()
            cost.backward()
            optimizer.step()
        optimized_trajectory = best_trajectory.detach().numpy()
        smooth_path = optimized_trajectory.reshape(-1, 2)
        smooth_path[0, :] = 0.  # TODO:Fix this ?add constraint to minimize?
        return smooth_path.astype(np.float64), best_cost

    def torch_normalize(self, v):
        # norm = np.linalg.norm(v, axis=0) + 0.00001
        norm = torch.norm(v, dim=0) + 0.00001
        return v / norm  # .reshape(1, v.shape[1])

    def torch_curvature(self, waypoints):
        '''
        Curvature as  the sum of the normalized dot product between the way elements
        Implement second term of the smoothing objective.

        args:
            waypoints [2, num_waypoints] !!!!!
        '''
        shift_left = torch.roll(waypoints, shifts=-1, dims=1)
        shift_right = torch.roll(waypoints, shifts=1, dims=1)
        left_half = self.torch_normalize(shift_left - waypoints)
        right_half = self.torch_normalize(waypoints - shift_right)
        mul = left_half * right_half
        segment = torch.sum(mul, dim=0)
        return torch.sum(segment[1:-1])

    def torch_smoothing_objective(self, waypoints, waypoints_center, weight_curvature=16):
        '''
        Objective for path smoothing

        args:
            waypoints [2 * num_waypoints] !!!!!
            waypoints_center [2 * num_waypoints] !!!!!
            weight_curvature (default=40)
        '''
        waypoints = waypoints.reshape(-1, 2)
        waypoints_center = waypoints_center.reshape(-1, 2)
        ls_tocenter = torch.sum(torch.square(waypoints - waypoints_center))
        ls_curvature = self.torch_curvature(waypoints.T)
        boundary_penalty = 0  # ? TODO? F_rddf in Stanley paper
        return ls_tocenter - weight_curvature * ls_curvature + boundary_penalty

    def numpy_normalize(self, v):
        norm = np.linalg.norm(v, axis=0) + 0.00001
        return v / norm

    def numpy_curvature(self, waypoints):
        '''
        Curvature as  the sum of the normalized dot product between the way elements
        Implement second term of the smoothing objective.

        args: 
            waypoints [2, num_waypoints] !!!!!
        '''
        shift_left = np.roll(waypoints, shift=-1, axis=1)
        shift_right = np.roll(waypoints, shift=1,  axis=1)
        left_half = self.numpy_normalize(shift_left - waypoints)
        right_half = self.numpy_normalize(waypoints - shift_right)
        mul = left_half * right_half
        segment = np.sum(mul, axis=0)
        return np.sum(segment[1:-1])

    def numpy_smoothing_objective(self, waypoints, waypoints_center, weight_curvature=16):
        '''
        Objective for path smoothing

        args:
            waypoints [2 * num_waypoints] !!!!!
            waypoints_center [2 * num_waypoints] !!!!!
            weight_curvature (default=40)
        '''
        waypoints = waypoints.reshape(-1, 2)
        waypoints_center = waypoints_center.reshape(-1, 2)
        ls_tocenter = np.sum(np.square(waypoints - waypoints_center))
        ls_curvature = self.numpy_curvature(waypoints.T)
        boundary_penalty = 0  # ? TODO? F_rddf in Stanley paper
        return ls_tocenter - weight_curvature * ls_curvature + boundary_penalty

    def scipy_smooth(self, path):
        opts = {"maxiter": 2, "disp": False}
        path_max = np.max(path)
        normalized_path = path / path_max
        way_points = minimize(self.numpy_smoothing_objective, (normalized_path),
                              args=normalized_path, options=opts, method='SLSQP', tol=1e-3)
        smooth_path = way_points["x"]
        smooth_path = smooth_path.reshape(-1, 2)
        smooth_path[0, :] = 0.  # TODO:Fix this ?add constraint to minimize?
        return (path_max*smooth_path).astype(np.float64), way_points["fun"]
