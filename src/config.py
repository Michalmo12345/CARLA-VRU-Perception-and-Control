import os

CLASS_MAPPING = {
    22: 0,        # Pedestrian
    1: 1, 2: 1,   # Bicycle / Bicyclist
    13: 2, 14: 2  # Motorcycle / Motorcyclist
}

IMAGE_WIDTH = 2048 
IMAGE_HEIGHT = 1024


SPLIT_RATIO = 0.8
IMAGE_EXT = '.png'
MIN_VISIBILITY = 0.3
RANDOM_SEED = 42

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ORIGINAL_JSON_DIR = os.path.join(BASE_DIR, 'bounding_box_2d')
ORIGINAL_IMAGES_DIR = os.path.join(BASE_DIR, 'rgb')
YOLO_DATASET_DIR = os.path.join(BASE_DIR, 'dataset')