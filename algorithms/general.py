import numpy as np


def global_to_local(pos, heading, path):
    car_heading = np.deg2rad(heading)
    R = np.array([[np.cos(car_heading), -np.sin(car_heading)],
                  [np.sin(car_heading), np.cos(car_heading)]])
    path -= pos
    path = (R.T @ path.T).T
    return path


def local_to_global(cones, car_pos, car_heading):
    car_heading = np.deg2rad(car_heading)

    R = np.array([[np.cos(car_heading), -np.sin(car_heading)],
                  [np.sin(car_heading), np.cos(car_heading)]])

    cones = (R @ cones.T).T
    cones[:, 0:2] += car_pos
    return cones


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


# def get_earth_radius_at_pos(lat: np.float64, sea_level: np.float64 = 0.0, ) -> np.float64:
    """
    Calculate the earth radius at the current origin given by latitude

    Args:
        lat (np.float64): latitude of current origin 
        sea_level (np.float64, optional): sea_level of the current position. Defaults to 0.
    Returns:
        np.float64: total earth radius at current position
    """
    #  Earth eccentricity e
#    e = np.array(0.0818, dtype=np.float64)
#    # Earth diameter at equator
#    r_equator = np.array(6378000, dtype=np.int64)
#    base_radius = r_equator * np.sqrt((1-(2*e**2-e**4)*np.sin(np.deg2rad(lat))**2)/(1-e**2*np.sin(np.deg2rad(lat))**2))
#    return base_radius + sea_level


# def lat_lon_to_meter_x_y(lat_lon_arr: np.ndarray[np.float64, np.float64], earth_radius: np.float64, lat_lon_origin: np.ndarray[np.float64, np.float64]) -> np.#ndarray[np.float64, np.float64]:
#    """
#    Converts the current position in degrees to position in meters based on origin array and earth radius at origin
#    Args:
#        lat_lon_arr (np.ndarray[np.float64, np.float64]): current position lat lon
#        earth_radius (np.float64): earth radius at origin
#        lat_lon_origin (np.ndarray[np.float64, np.float64]): lat lon at origin
#    Returns:
#        np.ndarray[np.float64, np.float64]: current position in meters from origin, order (x,y)
#    """
#    meter_yx = earth_radius * np.tan(np.deg2rad(lat_lon_arr - lat_lon_origin))
#    return meter_yx[::-1]
