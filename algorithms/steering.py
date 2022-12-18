import numpy as np
import math


def stanley_steering(path, speed, gain, lateralGain, max_range=22.5):
    index = len(path)-1
    if index > 10:
        index = 10

    index = min(1, len(path)-1)
    targ = path[index]
    dx = targ[1]
    dy = targ[0]
    direction = math.atan2(dy, dx)
    if len(path) > 1:
        latOffset = path[1][0]
    else:
        latOffset = 0
    if speed > 0.3:
        nonLinear = 2 * math.atan2(lateralGain * latOffset, speed) / np.pi
    else:
        nonLinear = 0
        # print("Not using non-linear")

    linear = gain * direction
    delta = linear + nonLinear
    delta *= 180/np.pi
    delta = np.clip(delta, -max_range, max_range)

    log_message = {"Linear": linear,
                   "Nonlinear": nonLinear,
                   "Lateral_offset": latOffset,
                   "Direction": direction,
                   "Delta": delta
                   }

    return delta, index, log_message
