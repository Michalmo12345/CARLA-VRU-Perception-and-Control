"""Planowanie trasy i geometria pasa/chodnika w CARLA.

Odpowiedzialność: znaleźć prostą trasę jazdy, miejsce spawnu auta oraz punkty
na chodnikach. Nie zajmuje się VRU ani sterowaniem — tylko mapa i waypointy.
"""

import math

import carla

from .config import (
    PEDESTRIAN_LANE_TYPES,
    PREFERRED_MAPS,
    ROUTE_MIN_M,
    ROUTE_STEP_M,
    TOWN02_ROUTE_ANCHOR,
)
from .geometry import yaw_delta


def _adjacent_lane(driving_wp, side: str):
    lane = driving_wp
    getter = lane.get_right_lane if side == "right" else lane.get_left_lane
    for _ in range(12):
        try:
            nxt = getter()
        except RuntimeError:
            return None, None
        if nxt is None:
            return None, None
        if nxt.lane_type in PEDESTRIAN_LANE_TYPES:
            return nxt, side
        if nxt.lane_type == carla.LaneType.Driving:
            lane = nxt
            getter = lane.get_right_lane if side == "right" else lane.get_left_lane
            continue
        lane = nxt
        getter = lane.get_right_lane if side == "right" else lane.get_left_lane
    return None, None


def _location_from_lane_wp(lane_wp, z_offset: float = 1.0) -> carla.Location:
    loc = lane_wp.transform.location
    return carla.Location(loc.x, loc.y, loc.z + z_offset)


def sidewalk_waypoint(driving_wp, street_side: str):
    """Chodnik względem kierunku jazdy (right = po prawej, left = naprzeciw)."""
    if street_side == "right":
        sw_wp, _ = _adjacent_lane(driving_wp, "right")
        return sw_wp

    try:
        opposite = driving_wp.get_left_lane()
    except RuntimeError:
        opposite = None
    if opposite is not None and opposite.lane_type == carla.LaneType.Driving:
        sw_wp, _ = _adjacent_lane(opposite, "right")
        if sw_wp is not None:
            return sw_wp
    sw_wp, _ = _adjacent_lane(driving_wp, "left")
    return sw_wp


def sidewalk_point(driving_wp, street_side: str) -> carla.Location:
    """Punkt na powierzchni chodnika (XY i Z z OpenDRIVE)."""
    sw_wp = sidewalk_waypoint(driving_wp, street_side)
    if sw_wp is not None:
        return _location_from_lane_wp(sw_wp, z_offset=1.0)

    tf = driving_wp.transform
    loc = tf.location
    right = tf.get_right_vector()
    lateral = 3.5 if street_side == "right" else 8.5
    sign = 1.0 if street_side == "right" else -1.0
    return carla.Location(
        loc.x + right.x * lateral * sign,
        loc.y + right.y * lateral * sign,
        loc.z + 1.0,
    )


def sidewalk_path_along_route(route, street_side: str, start_idx: int, end_idx: int):
    """Punkty na chodniku co waypoint trasy — chód wzdłuż ulicy, nie skrótem."""
    return [sidewalk_point(route[i], street_side) for i in range(start_idx, end_idx + 1)]


def route_has_sidewalks(route, samples: int = 5) -> bool:
    """Po obu stronach ulicy jest chodnik / pobocze."""
    if len(route) < 10:
        return False
    indices = [int(i * (len(route) - 1) / max(samples - 1, 1)) for i in range(samples)]
    for idx in indices:
        wp = route[idx]
        if sidewalk_waypoint(wp, "left") is None or sidewalk_waypoint(wp, "right") is None:
            return False
    return True


def waypoint_spawn_transform(waypoint, z_offset: float = 0.5) -> carla.Transform:
    """Waypoint ma z≈0 — podnieś aktora nad jezdnię, by ``try_spawn_actor`` zadziałał."""
    tf = waypoint.transform
    loc = carla.Location(tf.location.x, tf.location.y, tf.location.z + z_offset)
    return carla.Transform(loc, tf.rotation)


def route_waypoint_at(route, distance_m: float, step_m: float = ROUTE_STEP_M):
    idx = min(int(distance_m / step_m), len(route) - 1)
    return route[idx]


def find_route_from_anchor(
    carla_map,
    anchor: carla.Location,
    length_m: float,
    step_m: float = ROUTE_STEP_M,
    min_length_m: float = ROUTE_MIN_M,
):
    """Buduje prostą trasę przez znany punkt (łączy odcinek przed i za anchor)."""
    wp = carla_map.get_waypoint(anchor, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return None, None

    base_yaw = wp.transform.rotation.yaw
    max_steps = int(length_m / step_m) + 1

    backward = []
    current = wp
    while len(backward) < max_steps // 2:
        prev = current.previous(step_m)
        if not prev:
            break
        current = prev[0]
        if yaw_delta(current.transform.rotation.yaw, base_yaw) > 4.0:
            break
        backward.append(current)
    backward.reverse()

    forward = [wp]
    current = wp
    while len(forward) < max_steps:
        nxt = current.next(step_m)
        if not nxt:
            break
        current = nxt[0]
        if yaw_delta(current.transform.rotation.yaw, base_yaw) > 4.0:
            break
        forward.append(current)

    route = backward + forward
    if len(route) * step_m < min_length_m * 0.5:
        return None, None
    return waypoint_spawn_transform(route[0]), route


def find_straight_route(
    carla_map,
    length_m: float,
    step_m: float = ROUTE_STEP_M,
    min_length_m: float = ROUTE_MIN_M,
):
    """Skanuje waypoints OpenDRIVE i wybiera najdłuższą prostą linię."""
    best_route = None
    best_transform = None
    max_steps = int(length_m / step_m) + 1
    scan_step = step_m * 4

    for wp in carla_map.generate_waypoints(scan_step):
        if wp.lane_type != carla.LaneType.Driving:
            continue

        route = [wp]
        current = wp
        base_yaw = wp.transform.rotation.yaw

        while len(route) < max_steps:
            nxt = current.next(step_m)
            if not nxt:
                break
            current = nxt[0]
            if yaw_delta(current.transform.rotation.yaw, base_yaw) > 4.0:
                break
            route.append(current)

        if best_route is None or len(route) > len(best_route):
            best_route = route
            best_transform = waypoint_spawn_transform(route[0])

    return best_transform, best_route


def load_map_and_route(client, map_name: str, route_length_m: float, min_length_m: float):
    """Ładuje mapę i prostą trasę (Town02: znany odcinek po skręcie w lewo)."""
    candidates = list(PREFERRED_MAPS) if map_name == "auto" else [map_name]
    best = None

    for name in candidates:
        try:
            print(f"[*] Ładuję mapę: {name} ...")
            world = client.load_world(name)
            carla_map = world.get_map()

            if "Town02" in name:
                spawn_tf, route = find_route_from_anchor(
                    carla_map, TOWN02_ROUTE_ANCHOR, route_length_m, min_length_m=min_length_m,
                )
                if route:
                    length = len(route) * ROUTE_STEP_M
                    print(f"    Town02 prosta: {length:.0f} m (anchor y={TOWN02_ROUTE_ANCHOR.y:.0f})")
                    return name, world, spawn_tf, route

            spawn_tf, route = find_straight_route(carla_map, route_length_m, min_length_m=min_length_m)
            if not route:
                print("    brak trasy")
                continue
            length = len(route) * ROUTE_STEP_M
            has_sw = route_has_sidewalks(route)
            print(f"    prosta: {length:.0f} m | chodniki: {'tak' if has_sw else 'nie'}")
            if length >= min_length_m and has_sw:
                return name, world, spawn_tf, route
            if best is None or (has_sw and not best[5]) or (has_sw == best[5] and length > best[4]):
                best = (name, world, spawn_tf, route, length, has_sw)
        except RuntimeError as exc:
            print(f"    pominięto: {exc}")

    if best is not None:
        print(f"[!] Używam najlepszej dostępnej: {best[0]} ({best[4]:.0f} m, chodniki: {'tak' if best[5] else 'nie'})")
        return best[0], best[1], best[2], best[3]

    raise RuntimeError("Nie znaleziono mapy z prostą trasą")


def find_vehicle_spawn(world, carla_map, route, spawn_transform):
    """Szuka miejsca spawnu na trasie (waypoint + z) lub bliskim spawn point CARLA."""
    vehicle_bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    candidates = []

    if spawn_transform is not None:
        candidates.append(spawn_transform)
    for wp in route[:12]:
        candidates.append(waypoint_spawn_transform(wp))

    route_start = route[0].transform.location
    route_yaw = route[0].transform.rotation.yaw
    for sp in carla_map.get_spawn_points():
        dist = math.hypot(sp.location.x - route_start.x, sp.location.y - route_start.y)
        if dist > 40.0:
            continue
        if yaw_delta(sp.rotation.yaw, route_yaw) > 25.0:
            continue
        candidates.append(sp)

    for candidate in candidates:
        vehicle = world.try_spawn_actor(vehicle_bp, candidate)
        if vehicle is not None:
            world.tick()
            return vehicle
    return None
