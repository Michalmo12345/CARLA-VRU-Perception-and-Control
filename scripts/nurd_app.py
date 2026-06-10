import sys
import os
import time
import numpy as np
import cv2

# Project root setup
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from src.nurd_system.detection import DetectionModule
from src.nurd_system.tracking import TrackingModule
from src.nurd_system.distance import DistanceEstimationModule
from src.nurd_system.risk import CollisionModelParams, RiskAssessmentModule, RiskLevel

class NURDApp:
    """
    Integracja modułów NURD dla CARLA lub wideo.

    Parametry kamery (ogniskowa, rozmiar obrazu) są wstrzykiwane przez wywołującego
    — dla CARLA runner podaje realną ogniskową z FOV. Wartości domyślne to jedynie
    przybliżenie dla testowego trybu webcam (`main()`), gdzie FOV jest nieznane.
    """
    def __init__(
        self,
        model_path: str,
        focal_length_px: float = 320.0,
        image_width: int = 640,
        image_height: int = 640,
        base_speed: float = 50.0,
        fuse_closing_speed: bool = False,
    ):
        self.detector = DetectionModule(model_path=model_path)
        self.tracker = TrackingModule(min_hits=2)
        self.dist_module = DistanceEstimationModule(
            focal_length_px=focal_length_px,
            image_width=image_width,
            image_height=image_height,
        )
        self.risk_module = RiskAssessmentModule(
            base_speed=base_speed,
            focal_length_px=focal_length_px,
            image_width=image_width,
            params=CollisionModelParams(fuse_radial_closing=fuse_closing_speed),
        )

    def process_frame(self, frame: np.ndarray, dt: float, ego_speed_ms: float = None):
        """
        Główny pipeline: Detekcja -> Tracking -> Odległość -> Ryzyko
        """
        detections = self.detector.detect(frame)
        
        # tracks: [id, cx, cy, w, h, vx, vy, head, cls]
        tracks = self.tracker.update(detections, dt)
        
        if len(tracks) == 0:
            return frame, []

        # na podstawie bbox [id, cx, cy, w, h]
        dist_input = tracks[:, 0:5]
        class_ids = tracks[:, 8]
        distances = self.dist_module.estimate(dist_input, class_ids)

        kinematics = np.delete(tracks, [3, 4], axis=1)
        bboxes = tracks[:, 1:5]
        img_h, img_w = frame.shape[:2]
        risks = self.risk_module.assess(
            kinematics, distances, dt,
            bboxes=bboxes,
            image_size=(img_w, img_h),
            ego_speed_ms=ego_speed_ms,
        )

        for i, (track, risk) in enumerate(zip(tracks, risks)):
            tid, cx, cy, w, h, vx, vy, head, cls = track
            dist_val = distances[i].item()
            risk_lvl = risk['risk_level'].name
            v_app = risk['v_approach']
            on_path = risk.get('on_path', True)

            color = (0, 0, 255) if risk_lvl in ("HIGH", "CRITICAL") else (0, 255, 255) if risk_lvl == "MEDIUM" else (0, 255, 0)
            if not on_path:
                color = (128, 128, 128)
            
            cv2.rectangle(frame, (int(cx-w/2), int(cy-h/2)), (int(cx+w/2), int(cy+h/2)), color, 2)

            cv2.arrowedLine(frame, (int(cx), int(cy)), 
                            (int(cx + vx*15), int(cy + vy*15)), (255, 0, 0), 2)
            
            label = f"ID:{int(tid)} {dist_val:.1f}m {risk_lvl} v_app:{v_app:.1f}m/s"
            cv2.putText(frame, label, (int(cx-w/2), int(cy-h/2)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            risk['distance_m'] = dist_val

        risks_for_control = [
            r for r in risks
            if r.get("on_path", True) and r["risk_level"] != RiskLevel.LOW
        ]
        return frame, risks_for_control

def main():
    weights = 'runs/detect/TWM/run/weights/best.pt'
    if not os.path.exists(weights):
        # Fallback to base model if trained weights are missing
        weights = 'yolo11s.pt'
        print(f"[*] Nie znaleziono best.pt, używam modelu bazowego: {weights}")

    app = NURDApp(weights)

    # Tryb testowy na kamerze z PC. Pełna symulacja CARLA ma własny moduł:
    # nurd_carla_simulation.py → src/simulation/runner.py
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Błąd: Nie można otworzyć kamery.")
        return

    print("[*] Uruchomiono tryb testowy kamery. Naciśnij 'q' aby wyjść.")
    last_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        dt = max(dt, 0.001)

        processed_frame, risks = app.process_frame(frame, dt)
        
        cv2.imshow("NURD - Test Lokalny (Kamera)", processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
