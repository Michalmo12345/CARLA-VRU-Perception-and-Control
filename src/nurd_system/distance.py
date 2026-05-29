import numpy as np
from typing import Dict, List, Tuple, Optional, Union

class DistanceEstimationModule:
    """
    Estymacja Z na bazie modelu Pinhole Camera.
    """

    def __init__(self, focal_length_px: float, image_width: int, image_height: int):
        self.focal_length = focal_length_px
        self.image_width = image_width
        self.image_height = image_height

        # Fizyczne wysokości obiektów (metry)
        self.class_height_priors = {
            0: 1.70,  # Pedestrian
            1: 1.65,  # Cyclist
            2: 1.50   # Moped
        }

    def estimate(self, tracking_matrix: np.ndarray, class_ids: np.ndarray) -> np.ndarray:
        """
        Z = (H_real * f) / h_pixel
        """
        if tracking_matrix.shape[0] == 0:
            return np.empty((0, 1))

        pixel_heights = tracking_matrix[:, 4]
        distances = []

        for i, cls_id in enumerate(class_ids):
            real_height = self.class_height_priors.get(int(cls_id), 1.6)
            distance = (real_height * self.focal_length) / max(pixel_heights[i], 1e-6)
            distances.append(distance)

        return np.array(distances).reshape(-1, 1)
