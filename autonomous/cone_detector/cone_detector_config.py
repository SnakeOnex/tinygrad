import numpy as np
from pathlib import Path

file_path = Path(__file__).parent

logging_opt = {
    "log_name": "cone_detector",
    "log_folder_name": "AS",
}

cone_detector_opt = {
    "weights": file_path / Path('weights/yolov3_tiny_640_cpu.pt'),
    # "weights": file_path / Path('weights/yolov3_cones_640_gpu.pt'),
    "image_size": 640,
    "conf_thresh": 0.5,
    "iou_thresh": 0.15,
    "min_wh_ratio": 0.45,
    "zoom_profile": [(0.725, 0.2625, 0.55, 0.525), (0.725, 0.7375, 0.55, 0.525)] 
}

cone_localizer_opt = {
    "hom_mat": np.load(file_path / Path("hom_mat.npy")),
    "occlusion_profile": [-6., 6., 2.5, 15.]
}

path_planner_opt = {
    "n_steps": 5
}

config = {
    "cone_detector_opt": cone_detector_opt,
    "cone_localizer_opt": cone_localizer_opt,
    "path_planner_opt": path_planner_opt,
    "logging_opt": logging_opt
}
