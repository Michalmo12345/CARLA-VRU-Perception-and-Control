"""Parsowanie argumentów CLI dla symulacji CARLA + NURD."""

import argparse
import os

from .config import (
    APPROACH_SECONDS,
    CRUISE_SPEED_KMH,
    CYCLIST_SPEED_KMH,
    DEFAULT_HOST,
    DEFAULT_PORT,
    FREE_SIDEWALK_PEDESTRIANS,
    HOOD_CROSSING_AHEAD_M,
    MAP_NAME,
    NUM_VRU,
    PAUSE_ON_LANE_S,
    ROUTE_LENGTH_M,
    WALK_SPEED_MPS,
)
from .weather import WEATHER_CHOICES


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="CARLA + NURD: detekcja VRU i hamowanie")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Adres serwera CARLA (IP Windowsa z WSL2)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--model",
        default="runs/detect/TWM/run/weights/best.pt",
        help="Ścieżka do wag YOLO",
    )
    parser.add_argument(
        "--no-brake",
        action="store_true",
        help="Tylko detekcja i wizualizacja, bez sterowania pojazdem",
    )
    parser.add_argument(
        "--cruise-speed",
        type=float,
        default=CRUISE_SPEED_KMH,
        help="Docelowa prędkość jazdy w km/h gdy brak zagrożenia",
    )
    parser.add_argument(
        "--sim-delta",
        type=float,
        default=0.1,
        help="Krok czasu symulacji w sekundach (większy = wolniejsza CARLA)",
    )
    parser.add_argument(
        "--vehicle-slowdown",
        type=float,
        default=-50.0,
        help="Autopilot: %% wolniej od limitu (np. -50 = połowa prędkości)",
    )
    parser.add_argument(
        "--pedestrian-speed",
        type=float,
        default=WALK_SPEED_MPS,
        help="Prędkość chodu pieszych w m/s (domyślnie ~1.3)",
    )
    parser.add_argument(
        "--crossing-min",
        type=float,
        default=6.0,
        help="Min. odstęp między scenami przejścia [s]",
    )
    parser.add_argument(
        "--crossing-max",
        type=float,
        default=15.0,
        help="Maks. odstęp między scenami przejścia [s]",
    )
    parser.add_argument(
        "--pedestrians",
        type=int,
        default=NUM_VRU,
        help="Liczba VRU na trasie (używana tylko sekwencja 3: pieszy → rower → pieszy)",
    )
    parser.add_argument(
        "--map",
        default=MAP_NAME,
        help="Mapa CARLA (domyślnie Town02_Opt) lub 'auto'",
    )
    parser.add_argument(
        "--route-length",
        type=float,
        default=ROUTE_LENGTH_M,
        help="Długość prostej trasy w metrach",
    )
    parser.add_argument(
        "--camera-view",
        choices=["driver", "hood"],
        default="hood",
        help="Kamera na aucie: hood = maska (domyślnie), driver = widok kierowcy",
    )
    parser.add_argument(
        "--no-sync-spectator",
        action="store_true",
        help="Wyłącz podążanie widoku CARLA za kamerą auta",
    )
    parser.add_argument(
        "--cyclist-speed",
        type=float,
        default=CYCLIST_SPEED_KMH,
        help="Prędkość rowerzysty w km/h (domyślnie 10)",
    )
    parser.add_argument(
        "--free-pedestrians",
        type=int,
        default=FREE_SIDEWALK_PEDESTRIANS,
        help="Piesi swobodnie chodzący po chodnikach (0 = wyłączone)",
    )
    parser.add_argument(
        "--crossing-ahead",
        "--crossing-distance",
        type=float,
        default=HOOD_CROSSING_AHEAD_M,
        dest="crossing_ahead",
        help="Metry przed autem, gdzie VRU wchodzi na jezdnię (alias: --crossing-distance)",
    )
    parser.add_argument(
        "--approach-seconds",
        type=float,
        default=APPROACH_SECONDS,
        help="Ile sekund VRU idzie z krawędzi na pas przed postojem (domyślnie 4)",
    )
    parser.add_argument(
        "--pause-on-lane",
        type=float,
        default=PAUSE_ON_LANE_S,
        help="Ile sekund VRU stoi na pasie przed zejściem (domyślnie 2)",
    )
    parser.add_argument(
        "--fuse-closing-speed",
        action="store_true",
        help="Dołóż radialną prędkość VRU (z obrazu) do prędkości auta przy ocenie "
             "zbliżania. Domyślnie off — używana jest tylko dokładna prędkość auta.",
    )
    parser.add_argument(
        "--weather",
        choices=WEATHER_CHOICES,
        default="default",
        help="Pogoda: clear/cloudy/wet/rain/fog/night/rainnight (default = bez zmian)",
    )
    parser.add_argument(
        "--end-sign",
        action="store_true",
        help="Postaw znak drogowy w pasie pod koniec trasy (test reakcji na obiekt "
             "spoza klas YOLO — auto go nie wykryje)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Bez okna OpenCV w WSL (okno CARLA nadal pokazuje kamerę auta)",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Zatrzymaj po N klatkach (0 = bez limitu)",
    )
    return parser.parse_args(argv)


def resolve_model_path(model_path: str) -> str:
    if os.path.exists(model_path):
        return model_path
    fallback = "yolo11s.pt"
    print(f"[*] Nie znaleziono {model_path}, używam: {fallback}")
    return fallback
