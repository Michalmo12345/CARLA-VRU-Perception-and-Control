"""Czyste funkcje geometryczne na obiektach ``carla.Location`` / kątach yaw.

Brak zależności od świata CARLA — łatwe do testowania w izolacji.
"""

import math

import carla


def yaw_delta(a: float, b: float) -> float:
    """Najmniejsza różnica kątów yaw w stopniach (0–180)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def yaw_from_vector(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(dy, dx))


def planar_distance(a: carla.Location, b: carla.Location) -> float:
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def location_distance(a: carla.Location, b: carla.Location) -> float:
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) ** 0.5


def lerp_location(start: carla.Location, end: carla.Location, t: float) -> carla.Location:
    return carla.Location(
        x=start.x + (end.x - start.x) * t,
        y=start.y + (end.y - start.y) * t,
        z=start.z + (end.z - start.z) * t,
    )


def point_back_from(near: carla.Location, far: carla.Location, back_m: float) -> carla.Location:
    """Punkt ``back_m`` metrów od ``near`` w stronę ``far``."""
    total = location_distance(near, far)
    if total < 0.1:
        return far
    t = min(1.0, back_m / total)
    return lerp_location(near, far, t)


def lateral_offset(loc: carla.Location, waypoint) -> float:
    """Odległość boczna ``loc`` od środka pasa (znak: prawo +)."""
    tf = waypoint.transform
    right = tf.get_right_vector()
    center = tf.location
    return (loc.x - center.x) * right.x + (loc.y - center.y) * right.y
