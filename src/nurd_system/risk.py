import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np


class RiskLevel(Enum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    CRITICAL = 3


@dataclass
class _TrackMemory:
    """Wygładzony stan pojedynczego śledzonego VRU (filtr szumu pomiaru)."""
    distance: float
    closing_speed: float = 0.0
    hits: int = 1


@dataclass
class CollisionModelParams:
    """Parametry fizycznego modelu kolizji.

    Wydzielone do obiektu, by progi były jawne i łatwe do strojenia/testów
    (zamiast magic numbers rozsianych po logice).
    """
    corridor_half_width_m: float = 1.6   # połowa korytarza auta + margines
    stop_margin_m: float = 3.0           # zatrzymaj się tyle metrów przed VRU
    reaction_time_s: float = 0.3         # opóźnienie reakcji układu
    comfort_decel: float = 2.0           # komfortowe opóźnienie [m/s^2]
    hard_decel: float = 4.5              # twarde opóźnienie [m/s^2]
    critical_ttc_s: float = 1.5
    safe_ttc_s: float = 4.0
    min_approach_ms: float = 0.3         # poniżej tego nie uznajemy zbliżania
    ema_distance: float = 0.4            # waga nowego pomiaru dystansu
    ema_speed: float = 0.3               # waga nowego pomiaru prędkości
    lateral_horizon_s: float = 2.5       # horyzont predykcji ruchu bocznego
    max_vru_closing_ms: float = 8.0      # ile VRU może dokładać do prędkości auta
    fuse_radial_closing: bool = False    # dołóż radialną do prędkości auta (domyślnie off)


class RiskAssessmentModule:
    """Ocena ryzyka kolizji z VRU na podstawie geometrii kamery i kinematyki.

    Zasada działania:
    - dystans estymowany z bboxa jest wygładzany (EMA), bo jest źródłem szumu;
    - korytarz kolizji liczony w metrach (model pinhole), więc cały kadr jest
      analizowany, ale reagujemy tylko gdy tor VRU realnie przecina tor auta;
    - TTC i wymagane opóźnienie liczone z prędkości auta (dokładnej), a nie z
      szumnej prędkości radialnej — to eliminuje fałszywe CRITICAL;
    - poziom ryzyka wynika z wymaganego opóźnienia (płynne narastanie reakcji).
    """

    def __init__(
        self,
        base_speed: float = 50.0,
        focal_length_px: float = 640.0,
        image_width: int = 1280,
        params: Optional[CollisionModelParams] = None,
    ):
        self.base_speed = base_speed
        self.focal_length_px = focal_length_px
        self.image_width = image_width
        self.params = params or CollisionModelParams()
        self._memory: Dict[int, _TrackMemory] = {}

    # --- geometria -------------------------------------------------------
    def _lateral_offset_m(self, cx: float, distance: float) -> float:
        """Poprzeczne przesunięcie VRU od osi kamery w metrach (pinhole)."""
        cx0 = self.image_width / 2.0
        return (cx - cx0) * distance / max(self.focal_length_px, 1e-6)

    def _in_collision_corridor(self, x_now: float, x_future: float) -> bool:
        half = self.params.corridor_half_width_m
        if abs(x_now) <= half or abs(x_future) <= half:
            return True
        return x_now * x_future < 0.0  # tor przecina oś auta

    # --- kinematyka ------------------------------------------------------
    def _update_memory(self, tid: int, distance: float, dt: float) -> _TrackMemory:
        prev = self._memory.get(tid)
        if prev is None:
            mem = _TrackMemory(distance=distance)
            self._memory[tid] = mem
            return mem

        a = self.params.ema_distance
        smoothed = a * distance + (1 - a) * prev.distance
        raw_closing = (prev.distance - smoothed) / dt if dt > 1e-6 else 0.0
        b = self.params.ema_speed
        prev.closing_speed = b * raw_closing + (1 - b) * prev.closing_speed
        prev.distance = smoothed
        prev.hits += 1
        return prev

    def _closing_speed(self, ego_speed_ms: Optional[float], radial_ms: float) -> float:
        """Prędkość zbliżania = prędkość auta + ewentualny ruch VRU ku autu.

        Prędkość auta (z CARLA) jest „podłogą” — eliminuje niedoszacowanie przy
        stojącym VRU. Radialna (z obrazu, szumna) może ją podbić, ale tylko do
        fizycznie sensownego limitu, by pojedynczy szum dystansu nie zawyżał TTC.
        """
        if ego_speed_ms is None:
            return radial_ms
        if not self.params.fuse_radial_closing:
            return ego_speed_ms
        capped_radial = min(radial_ms, ego_speed_ms + self.params.max_vru_closing_ms)
        return max(ego_speed_ms, capped_radial)

    def _required_decel(self, approach_ms: float, distance: float) -> float:
        p = self.params
        available = distance - p.stop_margin_m - approach_ms * p.reaction_time_s
        if available <= 0.05:
            return float("inf")
        return (approach_ms ** 2) / (2.0 * available)

    def _target_speed_kmh(self, distance: float) -> float:
        """Płynny profil prędkości: maleje do 0 przy ``stop_margin``."""
        usable = max(distance - self.params.stop_margin_m, 0.0)
        v_ms = math.sqrt(2.0 * self.params.comfort_decel * usable)
        return min(self.base_speed, v_ms * 3.6)

    def _classify(self, in_path: bool, approaching: bool, ttc: float, a_req: float, distance: float):
        p = self.params
        if not in_path or not approaching:
            return RiskLevel.LOW, self.base_speed
        if a_req >= p.hard_decel or ttc < p.critical_ttc_s:
            return RiskLevel.CRITICAL, 0.0

        target = self._target_speed_kmh(distance)
        if a_req >= p.comfort_decel or ttc < p.safe_ttc_s:
            return RiskLevel.HIGH, target
        if target < self.base_speed * 0.95:
            return RiskLevel.MEDIUM, target
        return RiskLevel.LOW, self.base_speed

    # --- API -------------------------------------------------------------
    def assess(
        self,
        kinematics: np.ndarray,
        distances: np.ndarray,
        dt: float,
        bboxes: Optional[np.ndarray] = None,
        image_size: Optional[Tuple[int, int]] = None,
        ego_speed_ms: Optional[float] = None,
    ) -> List[Dict]:
        """Zwraca listę ocen ryzyka per track.

        ``ego_speed_ms`` (prędkość auta z CARLA) jest preferowanym źródłem
        prędkości zbliżania; gdy brak (np. tryb webcam), używamy prędkości
        radialnej z wygładzonego dystansu.
        """
        if image_size:
            self.image_width = image_size[0]

        results = []
        seen_ids = set()

        for i in range(len(distances)):
            tid = int(kinematics[i, 0])
            seen_ids.add(tid)
            mem = self._update_memory(tid, distances[i].item(), dt)
            distance = mem.distance
            radial = max(mem.closing_speed, 0.0)
            approach = self._closing_speed(ego_speed_ms, radial)
            approaching = approach >= self.params.min_approach_ms

            in_path = True
            lateral = 0.0
            if bboxes is not None and len(bboxes) > i and self.image_width > 0:
                cx, _, _, _ = bboxes[i]
                vx = kinematics[i, 3] if kinematics.shape[1] > 3 else 0.0
                lateral = self._lateral_offset_m(cx, distance)
                lateral_speed = self._lateral_offset_m(cx + vx, distance) - lateral
                ttc_for_pred = distance / approach if approach > 0.1 else 1.0
                horizon = min(max(ttc_for_pred, 0.5), self.params.lateral_horizon_s)
                in_path = self._in_collision_corridor(lateral, lateral + lateral_speed * horizon)

            ttc = distance / approach if approach > 0.1 else float("inf")
            a_req = self._required_decel(approach, distance) if approaching else 0.0
            lvl, v_target = self._classify(in_path, approaching, ttc, a_req, distance)

            results.append({
                "track_id": tid,
                "risk_level": lvl,
                "target_speed": v_target,
                "ttc_value": ttc,
                "v_approach": radial,
                "required_decel": a_req,
                "lateral_offset_m": lateral,
                "on_path": in_path,
            })

        self._memory = {tid: mem for tid, mem in self._memory.items() if tid in seen_ids}
        return results
