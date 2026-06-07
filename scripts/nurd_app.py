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
from src.nurd_system.risk import RiskAssessmentModule

class NURDApp:
    """
    Integracja modułów NURD dla CARLA lub wideo.
    """
    def __init__(self, model_path: str):
        self.detector = DetectionModule(model_path=model_path)
        self.tracker = TrackingModule(min_hits=2)
        # TODO: Kalibracja pod konkretny FOV kamery
        self.dist_module = DistanceEstimationModule(focal_length_px=320.0, image_width=640, image_height=640)
        self.risk_module = RiskAssessmentModule(base_speed=50.0)

    def process_frame(self, frame: np.ndarray, dt: float):
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
        risks = self.risk_module.assess(kinematics, distances, dt)

        for i, (track, risk) in enumerate(zip(tracks, risks)):
            tid, cx, cy, w, h, vx, vy, head, cls = track
            dist_val = distances[i].item()
            risk_lvl = risk['risk_level'].name
            v_app = risk['v_approach']

            color = (0, 0, 255) if risk_lvl in ['HIGH', 'CRITICAL'] else (0, 255, 0)
            
            cv2.rectangle(frame, (int(cx-w/2), int(cy-h/2)), (int(cx+w/2), int(cy+h/2)), color, 2)

            cv2.arrowedLine(frame, (int(cx), int(cy)), 
                            (int(cx + vx*15), int(cy + vy*15)), (255, 0, 0), 2)
            
            label = f"ID:{int(tid)} {dist_val:.1f}m {risk_lvl} v_app:{v_app:.1f}m/s"
            cv2.putText(frame, label, (int(cx-w/2), int(cy-h/2)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            risk['distance_m'] = dist_val

        return frame, risks

def main():
    weights = 'runs/detect/TWM/run/weights/best.pt'
    if not os.path.exists(weights):
        # Fallback to base model if trained weights are missing
        weights = 'yolo11s.pt'
        print(f"[*] Nie znaleziono best.pt, używam modelu bazowego: {weights}")

    app = NURDApp(weights)
    

    # Testowanie samego odpalenia i działania na kamerze z PC, dla Carli osobny moduł #TODO
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
