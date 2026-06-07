import numpy as np
from enum import Enum
from typing import Dict, List, Tuple, Optional, Union

class RiskLevel(Enum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    CRITICAL = 3

class RiskAssessmentModule:
    """
    Wyznaczanie TTC i poziomu zagrożenia.
    """

    def __init__(self, safe_ttc: float = 4.0, critical_ttc: float = 1.5, base_speed: float = 50.0):
        self.safe_ttc = safe_ttc
        self.critical_ttc = critical_ttc
        self.base_speed = base_speed
        
        
        self.distance_history: Dict[int, float] = {}
        self.v_radial_history: Dict[int, float] = {}
        self.smoothing_factor = 0.3 # EMA: 0.3 (nowy pomiar), 0.7 (historia)

    def _calc_ttc(self, dist: float, v_radial: float) -> float:
        # TTC liczymy tylko dla realnego zbliżania (> 0.2 m/s)
        if v_radial <= 0.2:
            return float('inf')
        return dist / v_radial

    def assess(self, kinematics: np.ndarray, distances: np.ndarray, dt: float) -> List[Dict]:
        """
        Główna logika decyzyjna z wygładzaniem prędkości.
        """
        results = []
        new_dist_history = {}
        new_v_history = {}
        
        for i in range(len(distances)):
            dist = distances[i].item()
            tid = int(kinematics[i, 0])
            
            # 1. Obliczenie surowej prędkości zbliżania
            v_raw = 0.0
            if tid in self.distance_history and dt > 0:
                v_raw = (self.distance_history[tid] - dist) / dt
            
            # 2. Wygładzanie (EMA Filter) - chroni przed skokami z szumu YOLO
            if tid in self.v_radial_history:
                v_smooth = (self.smoothing_factor * v_raw) + ((1 - self.smoothing_factor) * self.v_radial_history[tid])
            else:
                v_smooth = v_raw
            
            # Ograniczenie fizyczne (pieszy/rower w CARLA rzadko przekracza 30 m/s)
            v_smooth = np.clip(v_smooth, -50.0, 50.0)
            
            new_dist_history[tid] = dist
            new_v_history[tid] = v_smooth
            
            ttc = self._calc_ttc(dist, v_smooth)
            
            #ryzyko
            if ttc < self.critical_ttc or dist < 5.0:
                lvl = RiskLevel.CRITICAL
                v_target = 0.0
            elif ttc < self.safe_ttc:
                lvl = RiskLevel.HIGH
                v_target = self.base_speed * 0.3
            elif dist < 20.0:
                lvl = RiskLevel.MEDIUM
                v_target = self.base_speed * 0.7
            else:
                lvl = RiskLevel.LOW
                v_target = self.base_speed

            results.append({
                "track_id": tid,
                "risk_level": lvl,
                "target_speed": v_target,
                "ttc_value": ttc,
                "v_approach": v_smooth
            })
            
        self.distance_history = new_dist_history
        self.v_radial_history = new_v_history
        return results
