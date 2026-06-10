"""Spawn VRU (pieszy / rowerzysta) i ustawienie ich na nawierzchni jezdni.

Najważniejszy detal: po udanym spawnie wysokość Z aktora jest wyrównywana do
podłoża na podstawie jego bounding boxa, a nie sztywnej stałej. Dzięki temu
piesi nie zapadają się pod jezdnię ani nie unoszą się nad nią.
"""

import random

import carla

from .config import BIKE_ROAD_Z, PAUSE_ON_LANE_S, WALKER_ROAD_Z
from .crossing import snap_to_driving_surface
from .geometry import lerp_location
from .route_planner import sidewalk_path_along_route
from .vru_movement import LanePauseCrossing, WalkerLaneCrossing, WalkerPathPatrol


def _surface_z(world, loc: carla.Location) -> float:
    """Wysokość nawierzchni jezdni pod danym punktem."""
    wp = world.get_map().get_waypoint(
        loc, project_to_road=True, lane_type=carla.LaneType.Driving,
    )
    return wp.transform.location.z if wp is not None else loc.z


def _ground_offset(actor, is_bike: bool) -> float:
    """Offset Z między pivotem aktora a podłożem (stopy / koła na jezdni).

    Pieszy ma pivot ~w środku ciała, więc offset ≈ połowa wysokości bboxa.
    Pojazd (rower) ma pivot przy gruncie, więc offset ≈ 0. Gdy bbox jest
    niewiarygodny, używamy bezpiecznego fallbacku ze stałych konfiguracji.
    """
    box = actor.bounding_box
    offset = box.extent.z - box.location.z
    floor = 0.05 if is_bike else 0.6
    if offset < floor:
        return BIKE_ROAD_Z if is_bike else WALKER_ROAD_Z
    return offset


def _align_actor_to_surface(world, actor, loc: carla.Location, is_bike: bool) -> carla.Location:
    """Ustaw aktora dokładnie na nawierzchni jezdni i zwróć jego pozycję."""
    aligned = carla.Location(loc.x, loc.y, _surface_z(world, loc) + _ground_offset(actor, is_bike))
    actor.set_transform(carla.Transform(aligned))
    world.tick()
    return aligned


def _spawn_location_candidates(start, lane_stop, end):
    """Kolejne próby spawnu — od krawędzi w stronę środka pasa."""
    raw = [
        start,
        lerp_location(lane_stop, start, 0.75),
        lerp_location(lane_stop, start, 0.5),
        lerp_location(lane_stop, start, 0.25),
        lerp_location(lane_stop, end, 0.15),
        lane_stop,
    ]
    seen = set()
    out = []
    for loc in raw:
        key = (round(loc.x, 1), round(loc.y, 1))
        if key in seen:
            continue
        seen.add(key)
        out.append(loc)
    return out


def _try_spawn_at(world, blueprint, locations, is_bike: bool = False, snap_to_road: bool = True):
    """Próba spawnu (kilka offsetów Z, by ominąć kolizję z gruntem).

    ``snap_to_road`` rzutuje punkt na jezdnię (VRU przechodzące); dla pieszych na
    chodniku wyłączamy je, by spawnowali się tam, gdzie chodnik.
    """
    if not isinstance(locations, list):
        locations = [locations]

    for loc in locations:
        base = snap_to_driving_surface(world, loc, is_bike) if snap_to_road else loc
        for dz in (0.0, 0.2, 0.4, -0.1):
            spawn_loc = carla.Location(base.x, base.y, base.z + dz)
            actor = world.try_spawn_actor(blueprint, carla.Transform(spawn_loc))
            if actor is not None:
                return actor, spawn_loc
            world.tick()
    return None, None


def _try_spawn_walker(world, blueprint_library, locations, snap_to_road: bool = True):
    walker_bps = list(blueprint_library.filter("walker.pedestrian.*"))
    random.shuffle(walker_bps)
    for walker_bp in walker_bps:
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "true")
        actor, spawn_loc = _try_spawn_at(world, walker_bp, locations, is_bike=False, snap_to_road=snap_to_road)
        if actor is not None:
            return actor, spawn_loc
    return None, None


def _resolve_blueprints(blueprint_library, preferred_ids, fallback_filters):
    """Zwraca blueprinty wg preferowanych ID, z fallbackiem do filtrów rodziny."""
    candidates = []
    for bp_id in preferred_ids:
        try:
            candidates.append(blueprint_library.find(bp_id))
        except IndexError:
            pass
    if not candidates:
        for family in fallback_filters:
            candidates += list(blueprint_library.filter(family))
    return candidates


def _try_spawn_vehicle(world, blueprint_library, locations, preferred_ids, fallback_filters):
    for bp in _resolve_blueprints(blueprint_library, preferred_ids, fallback_filters):
        actor, spawn_loc = _try_spawn_at(world, bp, locations, is_bike=True)
        if actor is not None:
            return actor, spawn_loc
    return None, None


def _try_spawn_bike(world, blueprint_library, locations):
    return _try_spawn_vehicle(
        world, blueprint_library, locations,
        preferred_ids=("vehicle.diamondback.century", "vehicle.bh.crossbike"),
        fallback_filters=("vehicle.diamondback", "vehicle.bh"),
    )


def _try_spawn_motorcycle(world, blueprint_library, locations):
    return _try_spawn_vehicle(
        world, blueprint_library, locations,
        preferred_ids=("vehicle.harley-davidson.low_rider", "vehicle.yamaha.yzf", "vehicle.kawasaki.ninja"),
        fallback_filters=("vehicle.harley-davidson", "vehicle.yamaha", "vehicle.kawasaki"),
    )


def _build_teleport_crossing(world, actor, raw_start, lane_stop, end, speed: float, pause_s: float, is_bike: bool):
    """Ruch przez teleport transformu — dla pojazdów (rower), które nie mają
    animacji chodu. Fizyka wyłączona, bo pozycję ustawiamy ręcznie co klatkę."""
    actor.set_simulate_physics(False)
    world.tick()
    aligned_start = _align_actor_to_surface(world, actor, raw_start, is_bike)
    lane_stop = carla.Location(lane_stop.x, lane_stop.y, aligned_start.z)
    end = carla.Location(end.x, end.y, aligned_start.z)
    return LanePauseCrossing(aligned_start, lane_stop, end, speed, pause_s)


def spawn_pedestrian(world, blueprint_library, start, lane_stop, end, walk_speed: float, pause_s: float):
    locations = _spawn_location_candidates(start, lane_stop, end)
    walker, raw_start = _try_spawn_walker(world, blueprint_library, locations)
    if walker is None:
        raise RuntimeError("Nie udało się zespawnować pieszego")

    # Fizyka WŁĄCZONA: silnik gra animację chodu (WalkerControl) i trzyma stopy
    # na podłożu. Tylko jednorazowo wyrównujemy spawn do nawierzchni.
    _align_actor_to_surface(world, walker, raw_start, is_bike=False)
    crossing = WalkerLaneCrossing(lane_stop, end, walk_speed, pause_s)
    return walker, crossing


def spawn_cyclist(world, blueprint_library, start, lane_stop, end, ride_speed: float, pause_s: float):
    locations = _spawn_location_candidates(start, lane_stop, end)
    bike, raw_start = _try_spawn_bike(world, blueprint_library, locations)
    if bike is None:
        raise RuntimeError("Nie udało się zespawnować rowerzysty")
    crossing = _build_teleport_crossing(world, bike, raw_start, lane_stop, end, ride_speed, pause_s, is_bike=True)
    return bike, crossing


def spawn_motorcycle(world, blueprint_library, start, lane_stop, end, ride_speed: float, pause_s: float):
    locations = _spawn_location_candidates(start, lane_stop, end)
    moto, raw_start = _try_spawn_motorcycle(world, blueprint_library, locations)
    if moto is None:
        raise RuntimeError("Nie udało się zespawnować motocykla")
    crossing = _build_teleport_crossing(world, moto, raw_start, lane_stop, end, ride_speed, pause_s, is_bike=True)
    return moto, crossing


def spawn_vru(
    world, blueprint_library, vru_type: str,
    start, lane_stop, end,
    walk_speed: float, ride_speed: float, pause_s: float,
):
    """Fabryka VRU — wybiera spawner na podstawie typu."""
    if vru_type == "cyclist":
        return spawn_cyclist(world, blueprint_library, start, lane_stop, end, ride_speed, pause_s)
    if vru_type == "motorcycle":
        return spawn_motorcycle(world, blueprint_library, start, lane_stop, end, ride_speed, pause_s)
    return spawn_pedestrian(world, blueprint_library, start, lane_stop, end, walk_speed, pause_s)


def spawn_free_sidewalk_pedestrians(world, blueprint_library, route, count: int, walk_speed: float):
    """Piesi swobodnie spacerujący po chodnikach po obu stronach ulicy."""
    walkers = []
    patrols = []
    if count <= 0 or len(route) < 4:
        return walkers, patrols

    stride = max(2, len(route) // max(count, 1))
    for i in range(count):
        street_side = "right" if i % 2 == 0 else "left"
        start_idx = min(i * stride, len(route) - 2)
        end_idx = min(start_idx + stride + 4, len(route) - 1)
        path = sidewalk_path_along_route(route, street_side, start_idx, end_idx)
        if len(path) < 2:
            continue

        # Chodnik: nie rzutujemy na jezdnię i zostawiamy fizykę włączoną,
        # żeby pieszy szedł animowany (WalkerControl) i trzymał się chodnika.
        walker, _ = _try_spawn_walker(world, blueprint_library, path, snap_to_road=False)
        if walker is None:
            continue
        world.tick()
        walkers.append(walker)
        patrols.append(WalkerPathPatrol(path, walk_speed))

    return walkers, patrols
