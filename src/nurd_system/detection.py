import numpy as np
import torch
from ultralytics import YOLO
from typing import Optional

class DetectionModule:
    """
    Wrapper dla modelu YOLO dedykowany dla NURD.
    """

    def __init__(self, model_path: str, device: Optional[str] = None):
        if device is None:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device
            
        self.model = YOLO(model_path)
        self.model.to(self.device)

    def detect(self, frame: np.ndarray, conf_threshold: float = 0.50) -> np.ndarray:
        """
        Zwraca macierz detekcji [x1, y1, x2, y2, conf, cls].
        """
        results = self.model.predict(
            source=frame,
            conf=conf_threshold,
            iou=0.45,       
            agnostic_nms=True, 
            verbose=False,
            device=self.device
        )

        if not results or len(results[0].boxes) == 0:
            return np.empty((0, 6), dtype=np.float32)

        return results[0].boxes.data.cpu().numpy()
