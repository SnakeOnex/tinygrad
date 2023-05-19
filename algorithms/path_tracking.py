import numpy as np
import math

def get_lookahead_point(lookahead, path):
    """
    Given piecewise linear function and distance, returns a point that is that distance away on the line
    Args:
      lookahead - distance constant
      path - Nx2 numpy array
    Ret:
      target - 2D point
    """
    target = path[-1,:]

    cum_dist = 0.
    for i in range(0, path.shape[0]-1):
        line = path[i+1] - path[i]
        dist = np.linalg.norm(line)
        cum_dist += dist

        if cum_dist >= lookahead:
            last_piece_coef = (cum_dist - lookahead) / dist
            target = path[i+1] - last_piece_coef * line
            break
    return target

def stanley_steering(path, lookahead_dist, speed, gain, lateran_gain, max_range=95.):
    """
    Lateral steering controller
    Args:
      path - Nx2 numpy array representing the path the car should drive in car's coordinate system
      lookahead_dist - distance determining the lookahead_point on the path line
      speed - current speed of the car
      gain - controller parameter
      lateran_gain - controller parameter
      max_range - determines the output range of the controller to (-max_range, max_range)
    Ret:
      delta - steering wheel angle
      log_msg - dictionary containing internal values of the controller (for debugging purposes)
    """

    target = get_lookahead_point(lookahead_dist, path)

    dx = target[0]
    dy = target[1]
    direction = math.atan2(dy, dx)
    if len(path) > 1:
        lat_offset = path[1][1]
    else:
        lat_offset = 0
    if speed > 0.3:
        nonLinear = 2 * math.atan2(lateran_gain * lat_offset, speed) / np.pi
    else:
        nonLinear = 0

    linear = gain * direction
    delta = linear + nonLinear
    delta *= 180/np.pi
    delta = np.clip(delta, -max_range, max_range)

    log_message = {"linear": linear,
                   "non_linear": nonLinear,
                   "lateral_offset": lat_offset,
                   "target": target,
                   "direction": direction,
                   "delta": delta
                   }

    return delta, log_message
