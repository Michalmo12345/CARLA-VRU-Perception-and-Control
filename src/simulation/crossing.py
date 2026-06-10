"""Geometria przejść VRU po jezdni.

Liczy punkty: krawędź jezdni → środek pasa → druga krawędź, z poprawnym
offsetem czasowym (start tak daleko, by dojść na pas w ``approach_s`` sekund).
"""

import carla

from .config import BIKE_ROAD_Z, ROAD_EDGE_M, WALKER_ROAD_Z
from .geometry import lateral_offset, point_back_from
from .route_planner import route_waypoint_at


def road_lateral_point(waypoint, lateral_m: float, z_offset: float = WALKER_ROAD_Z) -> carla.Location:
    tf = waypoint.transform
    right = tf.get_right_vector()
    loc = tf.location
    return carla.Location(
        loc.x + right.x * lateral_m,
        loc.y + right.y * lateral_m,
        loc.z + z_offset,
    )


def snap_to_driving_surface(
    world, loc: carla.Location, is_bike: bool = False, ref_waypoint=None,
) -> carla.Location:
    """Zachowaj offset boczny względem pasa i ustaw Z tuż nad jezdnią."""
    z_off = BIKE_ROAD_Z if is_bike else WALKER_ROAD_Z
    wp = ref_waypoint
    if wp is None:
        wp = world.get_map().get_waypoint(
            loc, project_to_road=True, lane_type=carla.LaneType.Driving,
        )
    if wp is not None:
        lateral = lateral_offset(loc, wp)
        return road_lateral_point(wp, lateral, z_off)
    if loc.z > 0.01:
        return loc
    return carla.Location(loc.x, loc.y, z_off)


def crossing_points_from_waypoint(waypoint, from_side: str = "right"):
    """Przejście po jezdni: krawędź → środek pasa → druga krawędź (równe Z)."""
    sign = 1.0 if from_side == "right" else -1.0
    lane_stop = road_lateral_point(waypoint, 0.0)
    start = road_lateral_point(waypoint, ROAD_EDGE_M * sign)
    end = road_lateral_point(waypoint, -ROAD_EDGE_M * sign)
    return start, lane_stop, end


def crossing_points_timed(waypoint, from_side: str, speed_mps: float, approach_s: float):
    """Start tak daleko, by po ``approach_s`` sekundach dojść na pas."""
    start_edge, lane_stop, end_edge = crossing_points_from_waypoint(waypoint, from_side)
    approach_dist = speed_mps * approach_s
    start = point_back_from(lane_stop, start_edge, approach_dist)
    return start, lane_stop, end_edge


def crossing_at_route_distance(
    route, distance_m: float, from_side: str, speed_mps: float, approach_s: float,
    is_bike: bool = False,
):
    """Trzy punkty przejścia przypięte do trasy (start / postój / zejście)."""
    wp = route_waypoint_at(route, distance_m)
    start, lane_stop, end = crossing_points_timed(wp, from_side, speed_mps, approach_s)
    z_off = BIKE_ROAD_Z if is_bike else WALKER_ROAD_Z
    return (
        road_lateral_point(wp, lateral_offset(start, wp), z_off),
        road_lateral_point(wp, 0.0, z_off),
        road_lateral_point(wp, lateral_offset(end, wp), z_off),
    )
