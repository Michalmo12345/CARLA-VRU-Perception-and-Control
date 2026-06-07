import sys
import os
import numpy as np

# Dodanie ścieżki projektu
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from src.nurd_system.distance import DistanceEstimationModule
from src.nurd_system.risk import RiskAssessmentModule

def test_nurd_logic():
    """
    Samodzielny test weryfikujący logikę matematyczną i fizyczną systemu NURD.
    """
    print("[*] Uruchamianie testu logicznego (Distance + Risk)...")

    # 1. Inicjalizacja modułów
    dist_module = DistanceEstimationModule(focal_length_px=320.0, image_width=640, image_height=640)
    risk_module = RiskAssessmentModule(base_speed=50.0)

    # 2. Mock danych śledzenia: [track_id, cx, cy, w, h]
    mock_tracking = np.array([
        [101, 320, 320, 20, 40],   # Pieszy
        [102, 320, 320, 60, 100],  # Rowerzysta
        [103, 320, 320, 150, 250]  # Motorower
    ], dtype=np.float32)
    
    class_ids = np.array([0, 1, 2]) # [Pieszy, Rowerzysta, Motorower]

    # 3. Estymacja odległości
    est_distances = dist_module.estimate(mock_tracking, class_ids)
    
    # 4. Mock kinematyki: [track_id, cx, cy, vx, vy, head, class_id]
    mock_kinematics = np.array([
        [101, 320, 320, 0.5, 0, 0, 0],
        [102, 320, 320, 5.0, 0, 0, 1],
        [103, 320, 320, 15.0, 0, 0, 2]
    ], dtype=np.float32)

    # 5. Krok 1: Inicjalizacja historii
    dt = 0.1 # 10 FPS
    risk_results = risk_module.assess(mock_kinematics, est_distances, dt)

    print("\n[Krok 1: Stan początkowy]")
    print(f"{'ID':<5} | {'Dystans':<10} | {'TTC':<8} | {'Ryzyko':<10} | {'V_approach'}")
    print("-" * 65)
    for res, dist in zip(risk_results, est_distances):
        d_val = dist.item()
        ttc = res['ttc_value']
        ttc_str = f"{ttc:.2f}s" if ttc != float('inf') else "brak"
        print(f"{res['track_id']:<5} | {d_val:>8.2f}m | {ttc_str:<8} | {res['risk_level'].name:<10} | {res['v_approach']:.2f} m/s")

    # 6. Krok 2: Symulacja zbliżenia (o 1 metr w ciągu 0.1s -> 10 m/s)
    print("\n[Krok 2: Obiekty zbliżyły się o 1m w czasie 0.1s]")
    est_distances_step2 = est_distances - 1.0
    risk_results_step2 = risk_module.assess(mock_kinematics, est_distances_step2, dt)
    
    print(f"{'ID':<5} | {'Dystans':<10} | {'TTC':<8} | {'Ryzyko':<10} | {'V_approach'}")
    print("-" * 65)
    for res, dist in zip(risk_results_step2, est_distances_step2):
        d_val = dist.item()
        ttc = res['ttc_value']
        ttc_str = f"{ttc:.2f}s" if ttc != float('inf') else "brak"
        print(f"{res['track_id']:<5} | {d_val:>8.2f}m | {ttc_str:<8} | {res['risk_level'].name:<10} | {res['v_approach']:.2f} m/s")
    
    print("\n[+] Test logiczny zakończony.")

if __name__ == "__main__":
    test_nurd_logic()
