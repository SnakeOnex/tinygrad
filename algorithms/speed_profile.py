import torch
import numpy as np
import matplotlib.pyplot as plt
import math


# speed profile by Dima Khursenko (2023)
# based on Michal Horachek b.thesis (2022)

# Results:
# new(old version of my path plan) + my speed profile(np.max(arr)) = 30s (3 hitted cones)
# new(old version of my path plan) + old profile(const 5 m/s) = 40s (2 hitted cones)

# old path plan + my speed profile (np.max(arr)) = 36s (3 hitted cones)
# old path plan + old profile(const 5 m/s) = 45s (0 hitted cones)


# constants figuring in friction ellipse
CAR_MU = 0.8
GRAVITY = 9.8
MAX_ACCELERATION = 2    # TODO: ask for better values
MAX_BRAKING = 4       # this is actually F on the left side, F = m*a where m is the weight of car


class SpeedProfile():

    def __init__(self):
        self.safe_max_speed = 5.75  # precomputed using corner speed formula by Dima
        self.constant_speed = 5  # 5 m/s is the old speed profile

    # Computes speed profile for a path using the algorithm described in the stanford Ph.D. thesis

    def stanford_profile(self, seglengths, segcurvatures, initial_speed=None, bounded=False, const_speed=False):

        def constant_speed_profile():
            # constant speed profile with accelaration at the start # like old formula's speed

            def pass_1():
                ret = torch.sqrt((CAR_MU * GRAVITY) / segcurvatures)
                ret = torch.ones(len(ret)) * self.constant_speed
                return ret

            def pass_2(first_phase):
                if initial_speed == 0:
                    return second_pass(first_phase)
                else:
                    return first_phase

            first_phase = pass_1()
            second_phase = pass_2(first_phase)

            return second_phase

        def first_pass():
            # returns speed at the edge of the car's friction ellipse
            ret = torch.sqrt((CAR_MU * GRAVITY) / segcurvatures)

            # if len(ret) == 0:
            #     ret = np.array([self.safe_max_speed, self.safe_max_speed])

            if bounded:
                ret[-1] = self.safe_max_speed

            return ret

        def second_pass(first_pass):
            # modifies speed from first pass so that no too extreme acceleration is done
            if initial_speed is None:
                second_pass = first_pass.clone()  # autograd requires access to original profile tensor, so no inplace ops
            else:
                # print("Init speed:", initial_speed)
                second_pass = torch.tensor([initial_speed] + [0.] * (len(first_pass) - 1), dtype=torch.float64, requires_grad=False)

            start_idx = 0
            if initial_speed == 0:
                start_idx = 1

            # for s in range(1, first_pass.shape[0]):
            for s in range(start_idx, first_pass.shape[0]):   # ??? start from 0 or 1 ??? what about at [0] index???
                compensation = 2 * MAX_ACCELERATION * seglengths[s - 1]
                # remove clone() and autograd FUCKING DIES!!!
                second_pass[s] = torch.minimum(
                    first_pass[s],
                    torch.sqrt(second_pass[s - 1].clone() ** 2 + compensation)
                )

            # if initial_speed != 0:    # ??? Do I need this??
            #     second_pass[0] = torch.minimum(
            #                                     first_pass[0],
            #                                     torch.sqrt(initial_speed ** 2 + 2 * MAX_ACCELERATION * seglengths[0])
            #                                   )
            return second_pass

        def third_pass(second_pass):
            # modifies speed from second pass so that no too extreme braking is done
            third_pass = second_pass.clone()  # maybe empty_like(second_pass) and third_pass[0] = second_pass[0] would be faster

            for s in range(second_pass.shape[0] - 1, 0, -1):  # the range should be up to 0!!! (to -1 after changes the last index, but I don't want that!)
                compensation = 2 * MAX_BRAKING * seglengths[s]
                third_pass[s - 1] = torch.minimum(
                    second_pass[s - 1],
                    torch.sqrt(third_pass[s].clone() ** 2 + compensation)
                )
            return third_pass

        if len(segcurvatures) == 0:
            return torch.tensor([self.safe_max_speed])

        if const_speed:
            return constant_speed_profile()
        else:
            first = first_pass()
            second = second_pass(first)
            third = third_pass(second)

            # print("Phases:")
            # print(first)
            # print(second)
            # print(third)

            return third

    def lengths(self, path):
        # segment_length by Michal Horacek

        # REDO? torch -> numpy

        segment_length = torch.zeros(len(path))

        # path = torch.from_numpy(path) ###

        for i in range(len(segment_length)):
            # Puts the length of segment between i-th and (i+1)-th path point at position i

            # segment_length[i - 1] = torch.norm(path[i, :] - path[i - 1, :], p="fro")
            segment_length[i - 1] = np.linalg.norm(path[i, :] - path[i - 1, :])

        return segment_length

    def curvatures(self, path):
        # calculate_path_curvature by Michal Horacek
        # returns an approximation of path curvature using inscribed circles

        # path = torch.from_numpy(path) ###

        def curvature(origin, current, destination):
            # calculates radius of circle on whose circumference lie points origin, current and destination
            a = np.linalg.norm(current - origin)
            b = np.linalg.norm(destination - current)
            c = np.linalg.norm(destination - origin)

            q = (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)
            return (2 * np.sqrt(max(0., 1 - q**2))) / c

        segment_curvature = torch.zeros((path.shape[0], ))

        for i in range(path.shape[0]):
            segment_curvature[i] = curvature(path[i - 1, :], path[i, :], path[(i + 1) % path.shape[0], :])

        segment_curvature[0] = 0.
        segment_curvature[-1] = 0.

        # Same values as segment curvature but doesnt converge that fast. Maybe grad issues?
        # segment_curvature2 = torch.tensor([
        #     curvature(path[i-1, :], path[i, :], path[(i+1) % path.shape[0], :]) for i in range( len(path) )
        # ])

        return segment_curvature + 0.001  # the 1/10000 hack which just might save my thesis

    def compute_speed(self, path, init_speed, const_speed=False):
        # print("Path: ", path)
        # print("Init speed:", init_speed)
        SAFE_SPEED = 5.75

        if const_speed:
            my_speed_profile = self.stanford_profile(self.lengths(path), self.curvatures(path), initial_speed=init_speed, const_speed=True)
        else:
            my_speed_profile = self.stanford_profile(self.lengths(path), self.curvatures(path), initial_speed=init_speed, bounded=True)

        my_speed_profile = my_speed_profile.numpy()

        ret_speed = my_speed_profile[1] if len(my_speed_profile) > 1 else SAFE_SPEED
        # ret_speed = np.max(my_speed_profile) # quite aggressive?

        if math.isnan(ret_speed):
            ret_speed = SAFE_SPEED

        return round(ret_speed, 2), my_speed_profile

    def michals_profile(self, path, initial_speed=0.):
        # Computes speed profile for a path using the algorithm described in the stanford Ph.D. thesis
        def stanford_profile(seglengths, segcurvatures, initial_speed=None):
            # The Michal's function
            def first_pass():
                # returns speed at the edge of the car's friction ellipse
                return torch.sqrt((CAR_MU * GRAVITY) / segcurvatures)

            def second_pass(first_pass):
                # modifies speed from first pass so that no too extreme acceleration is done
                if initial_speed is None:
                    second_pass = first_pass.clone()  # autograd requires access to original profile tensor, so no inplace ops
                else:
                    second_pass = torch.tensor([initial_speed] + [0.] * (len(first_pass) - 1), dtype=torch.float64, requires_grad=False)

                for s in range(1, first_pass.shape[0]):
                    compensation = 2 * MAX_ACCELERATION * seglengths[s - 1]
                    # remove clone() and autograd FUCKING DIES!!!
                    second_pass[s] = torch.minimum(
                        first_pass[s],
                        torch.sqrt(second_pass[s - 1].clone() ** 2 + compensation)
                    )
                return second_pass

            def third_pass(second_pass):
                # modifies speed from second pass so that no too extreme braking is done
                third_pass = second_pass.clone()  # maybe empty_like(second_pass) and third_pass[0] = second_pass[0] would be faster
                for s in range(second_pass.shape[0] - 1, 0, -1):
                    compensation = 2 * MAX_BRAKING * seglengths[s]
                    third_pass[s - 1] = torch.minimum(
                        second_pass[s - 1],
                        torch.sqrt(third_pass[s].clone() ** 2 + compensation)
                    )
                return third_pass

            # constants figuring in friction ellipse
            CAR_MU = 0.5
            GRAVITY = 9.8
            MAX_ACCELERATION = 3    # TODO: ask for better values
            MAX_BRAKING = 6       # this is actually F on the left side, F = m*a where m is the weight of car

            first = first_pass()
            second = second_pass(first)
            third = third_pass(second)
            return third

        return stanford_profile(self.lengths(path), self.curvatures(path), initial_speed=initial_speed).numpy()
