import numpy as np


def get_earth_radius_at_pos(lat: np.float64, sea_level: np.float64 = 0.0, ) -> np.float64:
    """
    Calculate the earth radius at the current origin given by latitude

    Args:
        lat (np.float64): latitude of current origin 
        sea_level (np.float64, optional): sea_level of the current position. Defaults to 0.
    Returns:
        np.float64: total earth radius at current position
    """
    #  Earth eccentricity e
    e = np.array(0.0818, dtype=np.float64)
    # Earth diameter at equator
    r_equator = np.array(6378000, dtype=np.int64)
    base_radius = r_equator * np.sqrt((1 - (2 * e**2 - e**4) * np.sin(np.deg2rad(lat))**2) / (1 - e**2 * np.sin(np.deg2rad(lat))**2))
    return base_radius + sea_level


def lat_lon_to_meter_x_y(lat_lon_arr: np.ndarray[np.float64, np.float64], earth_radius: np.float64, lat_lon_origin: np.ndarray[np.float64, np.float64]) -> np.ndarray[np.float64, np.float64]:
    """
    Converts the current position in degrees to position in meters based on origin array and earth radius at origin
    Args:
        lat_lon_arr (np.ndarray[np.float64, np.float64]): current position lat lon
        earth_radius (np.float64): earth radius at origin
        lat_lon_origin (np.ndarray[np.float64, np.float64]): lat lon at origin
    Returns:
        np.ndarray[np.float64, np.float64]: current position in meters from origin, order (x,y)
    """
    meter_yx = earth_radius * np.tan(np.deg2rad(lat_lon_arr - lat_lon_origin))
    return meter_yx[::-1]
