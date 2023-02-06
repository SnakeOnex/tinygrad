import numpy as np


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
