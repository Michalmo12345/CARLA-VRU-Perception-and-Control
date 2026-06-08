"""Sekwencja scen VRU na trasie (pieszy → rower → pieszy).

Trzyma stan kolejnych przejść i decyduje, która scena jest następna oraz czy
poprzednia jeszcze trwa. Nie zna CARLA — operuje na czystych danych.
"""

from .config import SCENARIO_SEQUENCE


def build_route_crossings(route_length_m: float):
    """Mapuje ułamki trasy z konfiguracji na konkretne dystanse [m]."""
    crossings = []
    for item in SCENARIO_SEQUENCE:
        crossings.append({
            "label": item["label"],
            "vru_type": item["vru_type"],
            "distance_m": route_length_m * item["fraction"],
            "from_side": item["from_side"],
            "triggered": False,
            "actor": None,
            "manual_crossing": None,
        })
    return crossings


def next_scene(planned_crossings):
    for planned in planned_crossings:
        if not planned["triggered"]:
            return planned
    return None


def previous_scene_active(planned_crossings) -> bool:
    for planned in planned_crossings:
        if not planned["triggered"]:
            continue
        mc = planned.get("manual_crossing")
        if mc is not None and mc.active:
            return True
    return False


def scene_trigger_lead(cruise_speed_kmh: float, approach_s: float) -> float:
    """Metrów przed punktem sceny na trasie, kiedy startuje VRU."""
    car_mps = max(cruise_speed_kmh / 3.6, 1.0)
    return max(12.0, car_mps * approach_s + 6.0)
