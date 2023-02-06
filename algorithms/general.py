import numpy as np


def get_big_orange_distance(cone_preds, min_big_cones):
    """
    given cone predictions, returns distance of the car to mean position of big orange cones
    args:
      cone_preds - Nx3 np.array of cone positions [x,y,cls]
      min_big_cones - minimum count of big orange cones for the distance to be calculated
    rets:
      dist_to_finish
    """
    big_cone_idxs = np.where(cone_preds[:, 2] == 3)[0]
    if len(big_cone_idxs) >= min_big_cones:
        distance_to_finish = np.mean(cone_preds[big_cone_idxs, 0])
        return distance_to_finish
    else:
        return None


def get_orange_centerline(cone_preds):
    """
    Finds center points between 2 opposing cones, for finding the path in skidpad with only orange cones.
    Args:
        cone_preds (numpy.ndarray N x 3): cone positions [x,y,cls]
    Returns:
        numpy.ndarray [N/2 x 2]: center points between supplied cone positions
    """
    dists = np.sqrt(cone_preds[:, 0]**2 + cone_preds[:, 1]**2)
    cone_preds = cone_preds[np.argsort(dists, axis=0)]
    means = [[0., 0.]]
    for i in range(0, int(cone_preds.shape[0]/2), 2):
        means.append(np.mean(cone_preds[i:i+2, 0:2], axis=0))
    return np.array(means)
