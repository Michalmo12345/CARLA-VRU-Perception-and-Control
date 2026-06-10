"""Nakładka tekstowa na obraz kamery (status symulacji)."""

import cv2
import numpy as np


def draw_status_overlay(frame: np.ndarray, risks, brake_enabled: bool, speed_kmh: float):
    worst = min(risks, key=lambda r: r["target_speed"]) if risks else None

    status = "NURD+CARLA"
    if worst:
        status += f" | {worst['risk_level'].name} | cel: {worst['target_speed']:.0f} km/h"
    else:
        status += " | brak VRU"

    status += f" | {speed_kmh:.1f} km/h"
    status += " | HAMOWANIE ON" if brake_enabled else " | tylko detekcja"

    cv2.putText(
        frame, status, (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
    )
