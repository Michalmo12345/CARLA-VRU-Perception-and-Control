import numpy as np


def carla_image_to_bgr(image) -> np.ndarray:
    frame = np.frombuffer(image.raw_data, dtype=np.uint8)
    frame = frame.reshape((image.height, image.width, 4))
    return frame[:, :, :3]
