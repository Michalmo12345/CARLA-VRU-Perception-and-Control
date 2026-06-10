"""Strategie ruchu VRU (sterowane ręcznie, bez AI walkera).

Każda klasa realizuje wzorzec „strategia ruchu”: dostaje aktora i ``dt``,
ustawia jego transform. Pozycja Z jest ustalana przy spawnie (wyrównana do
podłoża), więc tutaj tylko interpolujemy między gotowymi punktami.
"""

import math

import carla

from .config import PAUSE_ON_LANE_S
from .geometry import lerp_location, location_distance, yaw_from_vector


class LanePauseCrossing:
    """Wejście na jezdnię → postój na pasie → zejście na drugą krawędź."""

    _APPROACH = "approach"
    _PAUSE = "pause"
    _EXIT = "exit"

    def __init__(
        self,
        start: carla.Location,
        lane_stop: carla.Location,
        end: carla.Location,
        speed_mps: float,
        pause_seconds: float = PAUSE_ON_LANE_S,
    ):
        self.start = start
        self.lane_stop = lane_stop
        self.end = end
        self.speed_mps = speed_mps
        self.pause_seconds = pause_seconds
        self.approach_dist = max(location_distance(start, lane_stop), 0.1)
        self.exit_dist = max(location_distance(lane_stop, end), 0.1)
        self.yaw = yaw_from_vector(end.x - start.x, end.y - start.y)
        self.phase = self._APPROACH
        self.segment_progress_m = 0.0
        self.pause_elapsed = 0.0
        self.active = False

    def begin(self):
        self.phase = self._APPROACH
        self.segment_progress_m = 0.0
        self.pause_elapsed = 0.0
        self.active = True

    def _place(self, actor, loc: carla.Location):
        actor.set_transform(carla.Transform(loc, carla.Rotation(yaw=self.yaw)))

    def update(self, actor, dt: float) -> bool:
        if not self.active or actor is None or not actor.is_alive:
            return False

        if self.phase == self._APPROACH:
            self.segment_progress_m += self.speed_mps * dt
            t = min(1.0, self.segment_progress_m / self.approach_dist)
            self._place(actor, lerp_location(self.start, self.lane_stop, t))
            if t >= 1.0:
                self.phase = self._PAUSE
                self.pause_elapsed = 0.0
            return False

        if self.phase == self._PAUSE:
            self._place(actor, self.lane_stop)
            self.pause_elapsed += dt
            if self.pause_elapsed >= self.pause_seconds:
                self.phase = self._EXIT
                self.segment_progress_m = 0.0
            return False

        self.segment_progress_m += self.speed_mps * dt
        t = min(1.0, self.segment_progress_m / self.exit_dist)
        self._place(actor, lerp_location(self.lane_stop, self.end, t))
        if t >= 1.0:
            self.active = False
            return True
        return False


class WalkerLaneCrossing:
    """Animowane przejście pieszego: krawędź → środek pasa → postój → zejście.

    W przeciwieństwie do teleportowania transformu (które dawało T-pose i wbijało
    nogi w grunt), sterujemy wbudowaną lokomocją pieszego przez ``WalkerControl``
    (kierunek + prędkość). Silnik gra animację chodu i trzyma postać na podłożu,
    więc fizyka aktora musi pozostać włączona. Pozycję mierzymy z aktora — nie
    interpolujemy jej ręcznie.
    """

    _WALK_TO_STOP = "to_stop"
    _PAUSE = "pause"
    _WALK_TO_END = "to_end"

    # Bezpiecznik: jeśli pieszy nie dojdzie w tym czasie, wymuś następną fazę.
    _WALK_TIMEOUT_S = 8.0

    def __init__(
        self,
        lane_stop: carla.Location,
        end: carla.Location,
        speed_mps: float,
        pause_seconds: float = PAUSE_ON_LANE_S,
        arrive_radius_m: float = 0.5,
    ):
        self.lane_stop = lane_stop
        self.end = end
        self.speed_mps = speed_mps
        self.pause_seconds = pause_seconds
        self.arrive_radius_m = arrive_radius_m
        self.phase = self._WALK_TO_STOP
        self.pause_elapsed = 0.0
        self.phase_elapsed = 0.0
        self.active = False

    def begin(self):
        self.phase = self._WALK_TO_STOP
        self.pause_elapsed = 0.0
        self.phase_elapsed = 0.0
        self.active = True

    @staticmethod
    def _planar_dist(a: carla.Location, b: carla.Location) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _walk_toward(self, actor, target: carla.Location):
        loc = actor.get_location()
        dx, dy = target.x - loc.x, target.y - loc.y
        norm = math.hypot(dx, dy)
        direction = carla.Vector3D(dx / norm, dy / norm, 0.0) if norm > 1e-3 else carla.Vector3D()
        actor.apply_control(carla.WalkerControl(direction=direction, speed=self.speed_mps, jump=False))

    @staticmethod
    def _stand(actor):
        actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(), speed=0.0, jump=False))

    def _arrived(self, actor, target: carla.Location) -> bool:
        if self._planar_dist(actor.get_location(), target) <= self.arrive_radius_m:
            return True
        return self.phase_elapsed > self._WALK_TIMEOUT_S

    def update(self, actor, dt: float) -> bool:
        if not self.active or actor is None or not actor.is_alive:
            return False
        self.phase_elapsed += dt

        if self.phase == self._WALK_TO_STOP:
            if self._arrived(actor, self.lane_stop):
                self._stand(actor)
                self.phase = self._PAUSE
                self.pause_elapsed = 0.0
                return False
            self._walk_toward(actor, self.lane_stop)
            return False

        if self.phase == self._PAUSE:
            self._stand(actor)
            self.pause_elapsed += dt
            if self.pause_elapsed >= self.pause_seconds:
                self.phase = self._WALK_TO_END
                self.phase_elapsed = 0.0
            return False

        if self._arrived(actor, self.end):
            self._stand(actor)
            self.active = False
            return True
        self._walk_toward(actor, self.end)
        return False


class WalkerPathPatrol:
    """Naturalny spacer pieszego po łańcuchu punktów (tam i z powrotem).

    Używa ``WalkerControl``, więc pieszy faktycznie idzie (animacja chodu),
    zamiast być teleportowany. Fizyka aktora musi być włączona.
    """

    _WALK_TIMEOUT_S = 12.0

    def __init__(self, points: list, speed_mps: float, arrive_radius_m: float = 0.6):
        self.points = points
        self.speed_mps = speed_mps
        self.arrive_radius_m = arrive_radius_m
        self.active = len(points) >= 2
        self.idx = 1
        self.forward = True
        self.phase_elapsed = 0.0

    @staticmethod
    def _planar_dist(a: carla.Location, b: carla.Location) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _walk_toward(self, actor, target: carla.Location):
        loc = actor.get_location()
        dx, dy = target.x - loc.x, target.y - loc.y
        norm = math.hypot(dx, dy)
        direction = carla.Vector3D(dx / norm, dy / norm, 0.0) if norm > 1e-3 else carla.Vector3D()
        actor.apply_control(carla.WalkerControl(direction=direction, speed=self.speed_mps, jump=False))

    def _advance_index(self):
        if self.forward:
            if self.idx + 1 >= len(self.points):
                self.forward = False
                self.idx = max(len(self.points) - 2, 0)
            else:
                self.idx += 1
        else:
            if self.idx - 1 < 0:
                self.forward = True
                self.idx = min(1, len(self.points) - 1)
            else:
                self.idx -= 1

    def update(self, actor, dt: float):
        if not self.active or actor is None or not actor.is_alive:
            return
        self.phase_elapsed += dt
        target = self.points[self.idx]
        reached = (
            self._planar_dist(actor.get_location(), target) <= self.arrive_radius_m
            or self.phase_elapsed > self._WALK_TIMEOUT_S
        )
        if reached:
            self.phase_elapsed = 0.0
            self._advance_index()
            return
        self._walk_toward(actor, target)

