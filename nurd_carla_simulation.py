"""Punkt wejścia: symulacja CARLA + NURD (detekcja VRU i hamowanie).

Cała logika żyje w pakiecie ``src/simulation``. Ten plik tylko sprawdza
dostępność API CARLA i uruchamia runner.

Uruchomienie (CARLA na Windows, klient w WSL2):
  python nurd_carla_simulation.py --host <IP_windowsa>

CARLA na tym samym hoście:
  python nurd_carla_simulation.py
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    import carla  # noqa: F401
except ImportError as exc:
    print(
        "Brak modułu 'carla'. Zainstaluj API z dystrybucji CARLA:\n"
        "  pip install <ścieżka>/PythonAPI/carla/dist/carla-*-py3.x-*.egg"
    )
    raise SystemExit(1) from exc

from src.simulation.runner import run


if __name__ == "__main__":
    run()
