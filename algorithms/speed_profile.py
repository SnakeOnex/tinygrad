import numpy as np

# speed profile by Dima Khursenko (2023)
# based on Michal Horachek bachelor's thesis (2022)

# constants figuring in friction ellipse
CAR_MU = 0.8
GRAVITY = 9.8
MAX_ACCELERATION = 2    # TODO: ask for better values
# max decelaration/brakanig should be a negative value, but in reality positive value works better
MAX_BRAKING = 4       # this is actually F on the left side, F = m*a where m is the weight of car

# best constants for simulator?
# CAR_MU = 0.5
# GRAVITY = 9.8
# MAX_ACCELERATION = 3    # TODO: ask for better values
# MAX_BRAKING = 6

class SpeedProfile():
    """ Three-pass speed profile. """
    
    def __init__(self, max_safe_speed=5.75):
        self.MAX_SAFE_SPEED = max_safe_speed  
        # self.MAX_SAFE_SPEED = 5.75 # precomputed using corner speed formula: v=sqrt(mu*(m*g+F_aero)*r/m)

        self.seg_curvatures = None
        self.seg_lengths = None

        self.initial_speed = 0.0

        self.first_pass = None
        self.second_pass = None
        self.third_pass = None
    
    def first_phase(self):
        # computes speed at the edge of the car's friction ellipse
        self.first_pass = np.hstack((
            self.initial_speed,
            np.sqrt((CAR_MU * GRAVITY) / self.seg_curvatures),
            self.MAX_SAFE_SPEED)).ravel()
    
    def second_phase(self):
        # modifies speed from first pass so that no too extreme acceleration is done
        self.second_pass = np.copy(self.first_pass)

        for s in range(1, len(self.first_pass)):
            # ? asi rozepsat MAX_acceleration? podle Kapania
            compensation = 2 * MAX_ACCELERATION * self.seg_lengths[s]
            self.second_pass[s] = np.minimum(self.first_pass[s],
                                        np.sqrt(self.second_pass[s-1] ** 2 + compensation))
    
    def third_phase(self):
        # modifies speed from second pass so that no too extreme braking is done
        self.third_pass = np.copy(self.second_pass)

        for s in range(len(self.second_pass) - 1, 1, -1):
            compensation = 2 * MAX_BRAKING * self.seg_lengths[s]
            self.third_pass[s-1] = np.minimum(
                                            self.second_pass[s-1],
                                            np.sqrt(self.second_pass[s] ** 2 + compensation)
                                            )

    def compute_speed_profile(self, path, initial_speed=0.0):
        self.seg_curvatures = curvatures(path)
        self.seg_lengths = lengths(path)

        self.initial_speed = initial_speed

        self.first_phase()
        self.second_phase()
        self.third_phase()

        return self.third_pass


def curvatures(path):
        # returns an approximation of path curvature using inscribed circles
        def curvature(p1, p2, p3):
            # calculates radius of circle on whose circumference lie points p1, p2 and p3
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p1 - p3)
            
            q = (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)
            return (2 * np.sqrt(1 - q**2)) / c

        segment_curvature = np.zeros((path.shape[0]-2,))
        for i in range(len(segment_curvature)):
            j = i+1
            segment_curvature[i] = curvature(path[j-1], path[j], path[j+1])
        
        return segment_curvature

def lengths(path):
    # compute segment length of a path
    shifted_path = np.vstack((np.array([0., 0.]), path))[0:-1]
    segment_length = np.linalg.norm(shifted_path - path, axis=1)

    return segment_length  
