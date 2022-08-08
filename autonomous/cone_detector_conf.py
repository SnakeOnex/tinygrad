cone_detector_opt = {
  "weights": Path('weights/weights_640.pt'),
  "image_size": 640,
  "conf_thresh": 0.5,
  "iou_thresh": 0.15,
  "min_wh_ratio": 0.45,
  "zoom_profile": [(0.725, 0.2625, 0.55, 0.525), (0.725, 0.7375, 0.55, 0.525)] 
}

cone_localizer_opt = {
    "hom_mat_path": np.load(Path("hom_mat.npy")),
    "occlusion_profile": [-6., 6., 2.5, 15.]
}

config = {
    "cone_detector_opt": cone_detector_opt,
    "cone_localizer_opt", cone_localizer_opt
}
