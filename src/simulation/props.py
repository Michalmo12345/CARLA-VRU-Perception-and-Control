"""Statyczne przeszkody na trasie (np. znak drogowy).

Służą do pokazania ograniczenia percepcji: obiekt spoza klas wytrenowanych w
YOLO (pieszy/rower/motocykl) nie zostanie wykryty, więc auto nie zareaguje.
"""

import carla

from .crossing import snap_to_driving_surface
from .route_planner import route_waypoint_at

# Kolejne próby — różne wersje znaków/oznaczeń drogowych między buildami CARLA.
SIGN_BLUEPRINT_IDS = (
    "static.prop.streetsign",
    "static.prop.streetsign01",
    "static.prop.streetsign04",
    "static.prop.trafficwarning",
    "static.prop.constructioncone",
    "static.prop.warningconstruction",
)


def spawn_road_sign(world, blueprint_library, route, distance_m: float):
    """Stawia znak na środku pasa we wskazanym miejscu trasy. Zwraca aktora lub None."""
    waypoint = route_waypoint_at(route, distance_m)
    center = waypoint.transform.location
    base = snap_to_driving_surface(world, carla.Location(center.x, center.y, center.z), is_bike=True)
    # Obrót o 90° względem osi pasa (znak ustawiony w poprzek, przodem do auta).
    lane_rotation = waypoint.transform.rotation
    rotation = carla.Rotation(
        pitch=lane_rotation.pitch,
        yaw=lane_rotation.yaw + 90.0,
        roll=lane_rotation.roll,
    )

    for bp_id in SIGN_BLUEPRINT_IDS:
        try:
            blueprint = blueprint_library.find(bp_id)
        except IndexError:
            continue
        for dz in (0.0, 0.3, 0.6):
            transform = carla.Transform(carla.Location(base.x, base.y, base.z + dz), rotation)
            actor = world.try_spawn_actor(blueprint, transform)
            if actor is not None:
                return actor
            world.tick()
    return None
