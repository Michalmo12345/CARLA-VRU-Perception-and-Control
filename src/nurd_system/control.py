import math
from typing import List, Dict, Optional, Tuple

from .risk import RiskLevel


class VehicleControlModule:
    """
    Mapuje decyzje modułu ryzyka na komendy sterowania pojazdem (CARLA VehicleControl).
    """

    # Maksymalna intensywność hamulca dla zwykłego zwalniania (nie-CRITICAL).
    # Pełny hamulec (1.0) jest zarezerwowany dla sytuacji krytycznych.
    MAX_REGULAR_BRAKE = 0.85
    # Ile km/h nadmiaru ponad cel = pełny (regularny) hamulec. Większa wartość
    # = łagodniejsze, bardziej proporcjonalne hamowanie.
    BRAKE_GAIN_KMH = 18.0
    # Strefa martwa wokół celu [km/h] — w niej ani gaz, ani hamulec (anty-szarpanie).
    SPEED_DEADBAND_KMH = 1.5
    # Wygładzanie komendy hamulca (low-pass). 0 = brak zmian, 1 = natychmiast.
    BRAKE_SMOOTHING = 0.5

    def __init__(self, cruise_speed_kmh: float = 30.0):
        self.cruise_speed_kmh = cruise_speed_kmh
        self._last_brake = 0.0

    @staticmethod
    def _speed_ms_to_kmh(speed_ms: float) -> float:
        return speed_ms * 3.6

    def select_worst_risk(self, risks: List[Dict]) -> Optional[Dict]:
        if not risks:
            return None
        return min(risks, key=lambda r: r["target_speed"])

    def _smooth_brake(self, raw_brake: float) -> float:
        """Low-pass na komendzie hamulca — eliminuje szarpanie między klatkami."""
        smoothed = (1 - self.BRAKE_SMOOTHING) * self._last_brake + self.BRAKE_SMOOTHING * raw_brake
        self._last_brake = smoothed
        return smoothed

    def compute_throttle_brake(
        self,
        current_speed_ms: float,
        risks: List[Dict],
    ) -> Tuple[float, float]:
        """Zwraca (throttle, brake) w zakresie 0.0–1.0.

        Anty-szarpanie:
        - gdy obecne jest jakiekolwiek ryzyko, nigdy nie dodajemy gazu (coast);
          to usuwa walkę gaz↔hamulec, która powodowała szarpanie;
        - strefa martwa wokół prędkości docelowej zapobiega mikro-oscylacjom;
        - komenda hamulca jest wygładzana w czasie (low-pass).

        Pełny hamulec tylko przy CRITICAL (realnie nie zdążymy zwolnić komfortowo).
        """
        current_kmh = self._speed_ms_to_kmh(current_speed_ms)
        worst = self.select_worst_risk(risks)

        if worst is None:
            # Brak ryzyka: zwalniamy hamulec płynnie i wracamy do jazdy.
            brake = self._smooth_brake(0.0)
            throttle = self._cruise_throttle(current_kmh) if brake < 0.05 else 0.0
            return throttle, (brake if brake >= 0.05 else 0.0)

        if worst["risk_level"] == RiskLevel.CRITICAL:
            self._last_brake = 1.0
            return 0.0, 1.0

        speed_error = current_kmh - worst["target_speed"]

        # W strefie martwej i poniżej celu: tocz się (bez gazu, bez hamulca),
        # jedynie miękko zwalniając ewentualny resztkowy hamulec.
        if speed_error <= self.SPEED_DEADBAND_KMH:
            brake = self._smooth_brake(0.0)
            return 0.0, (brake if brake >= 0.05 else 0.0)

        raw_brake = min(self.MAX_REGULAR_BRAKE, speed_error / self.BRAKE_GAIN_KMH)
        return 0.0, self._smooth_brake(raw_brake)

    def _cruise_throttle(self, current_kmh: float) -> float:
        if current_kmh >= self.cruise_speed_kmh:
            return 0.0
        return min(0.85, (self.cruise_speed_kmh - current_kmh) / max(self.cruise_speed_kmh, 1.0))

    @staticmethod
    def focal_length_from_fov(image_width: int, fov_degrees: float) -> float:
        return (image_width / 2.0) / math.tan(math.radians(fov_degrees / 2.0))

    @staticmethod
    def compute_route_steer(vehicle, route, distance_driven: float, step_m: float = 2.0) -> float:
        """Kieruje autem wzdłuż wcześniej zaplanowanej prostej trasy."""
        if not route:
            return 0.0

        vehicle_tf = vehicle.get_transform()
        v_loc = vehicle_tf.location
        lookahead_idx = min(len(route) - 1, int(distance_driven / step_m) + 4)
        target = route[lookahead_idx].transform.location

        yaw = math.radians(vehicle_tf.rotation.yaw)
        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)
        dx = target.x - v_loc.x
        dy = target.y - v_loc.y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            return 0.0

        cross = fwd_x * dy - fwd_y * dx
        return max(-0.35, min(0.35, cross / dist * 2.0))

    @staticmethod
    def compute_lane_steer(vehicle, world, lookahead: float = 4.0) -> float:
        """Lekka korekta kierownicy, żeby trzymać pas na prostej."""
        carla_map = world.get_map()
        vehicle_tf = vehicle.get_transform()
        wp = carla_map.get_waypoint(vehicle_tf.location, project_to_road=True)
        if wp is None:
            return 0.0

        next_wps = wp.next(lookahead)
        if not next_wps:
            return 0.0

        target = next_wps[0].transform.location
        v_loc = vehicle_tf.location
        yaw = math.radians(vehicle_tf.rotation.yaw)
        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)

        dx = target.x - v_loc.x
        dy = target.y - v_loc.y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            return 0.0

        cross = fwd_x * dy - fwd_y * dx
        return max(-0.45, min(0.45, cross / dist * 1.8))
