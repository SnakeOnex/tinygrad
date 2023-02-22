import numpy as np


def generate_center_rect(start, current, cone_preds):
    """
    Updates current center cone detection based on current prediction 
    Args:
        current (dict): current estimation of center cones
        cone_preds (np.ndarray): current big orange cone predictions
    """

    pass


def generate_perfect_map(center_rect):
    """
    Generates perfect skidpad map from detection of center big cone rectangle
    Args:
        center_rect (numpy.ndarray): points making up the center rectangle
    """

    pass


def update_map_from_detections(cone_detections, map):
    """
    Updates the current skidpad map with new cone detections
    Args:
        cone_detections (numpy.ndarray): all currently detected cones by the car
        map (dict): dictionary with cone positions in skidpad-specific coordinates
    """

    pass


def get_path_graph(map):
    """
    Computes and returns the driving path graph for the current
    Args:
        map (dict): dictionary with cone positions in skidpad-specific coordinates
    """

    pass
