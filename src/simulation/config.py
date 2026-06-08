"""Stałe konfiguracyjne symulacji CARLA + NURD.

Pojedyncze źródło prawdy dla parametrów mapy, trasy, kamery i scenariuszy VRU.
Dzięki temu logika (runner, spawner, planner) nie trzyma żadnych „magic numbers”.
"""

import carla

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 2000

MAP_NAME = "Town02_Opt"
PREFERRED_MAPS = (
    "Town02_Opt",
    "Town02",
    "Town10HD_Opt",
    "Town05_Opt",
)
# Prosta ulica w Town02 po skręcie w lewo (yaw ~90°, długi odcinek)
TOWN02_ROUTE_ANCHOR = carla.Location(x=-7.53, y=251.36, z=0.25)

# Kamera / obraz
IMG_W = 1280
IMG_H = 720
FOV = 90.0

# Trasa i jazda
ROUTE_LENGTH_M = 90.0
ROUTE_MIN_M = 80.0
ROUTE_STEP_M = 2.0
CRUISE_SPEED_KMH = 20.0

# VRU
NUM_VRU = 3
WALK_SPEED_MPS = 3.0
CYCLIST_SPEED_KMH = 10.0
APPROACH_SECONDS = 4.0
PAUSE_ON_LANE_S = 7.0
HOOD_CROSSING_AHEAD_M = 16.0
MIN_SCENE_GAP_M = 18.0
ROAD_EDGE_M = 3.0

# Domyślna wysokość Z (offset nad nawierzchnią) używana przy próbach spawnu
# oraz jako fallback, gdy bounding box aktora jest niewiarygodny.
# Ostateczna pozycja VRU jest wyrównywana do podłoża na podstawie bounding boxa.
WALKER_ROAD_Z = 0.9
BIKE_ROAD_Z = 0.3

# Piesi swobodnie spacerujący po chodnikach (0 = wyłączeni)
FREE_SIDEWALK_PEDESTRIANS = 20
FREE_WALK_SPEED_MPS = 1.2

# Sekwencja scen na trasie (stałe miejsca, wyzwalane po kolei).
# Kolejność wg rosnącego "fraction" — VRU pojawiają się jeden po drugim.
SCENARIO_SEQUENCE = (
    {"label": "pieszy 1", "vru_type": "pedestrian", "fraction": 0.16, "from_side": "right"},
    {"label": "rower", "vru_type": "cyclist", "fraction": 0.34, "from_side": "left"},
    {"label": "pieszy 2", "vru_type": "pedestrian", "fraction": 0.52, "from_side": "left"},
    {"label": "pieszy 3", "vru_type": "pedestrian", "fraction": 0.70, "from_side": "right"},
    {"label": "motocykl", "vru_type": "motorcycle", "fraction": 0.86, "from_side": "left"},
)

PEDESTRIAN_LANE_TYPES = (
    carla.LaneType.Sidewalk,
    carla.LaneType.Shoulder,
    carla.LaneType.Border,
)

# Widok z auta: driver = kabina, hood = maska / zderzak (domyślny)
CAMERA_PRESETS = {
    "driver": carla.Transform(
        carla.Location(x=0.4, z=1.35),
        carla.Rotation(pitch=-4.0),
    ),
    "hood": carla.Transform(
        carla.Location(x=2.15, z=1.05),
        carla.Rotation(pitch=-6.0),
    ),
}
