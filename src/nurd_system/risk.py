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
        
        # Ostatnie znane odległości dla obliczenia prędkości radialnej
        self.distance_history: Dict[int, float] = {}

    def _calc_ttc(self, dist: float, v_radial: float) -> float:
        if v_radial <= 0.1: # Brak zbliżania lub błąd pomiaru
            return float('inf')
        return dist / v_radial

    def assess(self, kinematics: np.ndarray, distances: np.ndarray, dt: float) -> List[Dict]:
        """
        Główna logika decyzyjna.
        """
        results = []
        new_history = {}
        
        for i in range(len(distances)):
            dist = distances[i].item()
            tid = int(kinematics[i, 0])
            
            # Prędkość zbliżania w m/s
            v_radial = 0.0
            if tid in self.distance_history and dt > 0:
                v_radial = (self.distance_history[tid] - dist) / dt
                
            new_history[tid] = dist
            ttc = self._calc_ttc(dist, v_radial)
            
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
                "v_approach": v_radial
            })
            
        self.distance_history = new_history
        return results
