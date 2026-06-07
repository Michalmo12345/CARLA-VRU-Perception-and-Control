import sys, os; sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import os
import json
import cv2
import numpy as np
from nurd_app import NURDApp

def get_3d_gt(scene_id, sensor_id, frame_index):
    folder_path = f"dataset/bounding_box_3d/{scene_id}/{sensor_id}"
    if not os.path.exists(folder_path): return None
    prefix = f"{int(frame_index):018d}"
    for f in os.listdir(folder_path):
        if f.startswith(prefix) and f.endswith(".json"):
            with open(os.path.join(folder_path, f), 'r') as jf:
                return json.load(jf)
    return None

def is_nurd(vehicle_type):
    vt = vehicle_type.lower()
    if "pedestrian" in vt: return 0
    if "bicycle" in vt or "cyclist" in vt: return 1
    if "motorcycle" in vt or "moped" in vt: return 2
    return -1

def validate_system():
    weights = 'runs/detect/TWM/run/weights/best.pt'
    if not os.path.exists(weights): weights = 'yolo11s.pt'
    
    app = NURDApp(weights)
    image_dir = "dataset/images/train"
    
    # Szukamy klatek, które mają dużo obiektów (niepuste)
    all_images = [f for f in os.listdir(image_dir) if f.endswith(".png")]
    np.random.shuffle(all_images)
    test_images = all_images[:100]

    print(f"{'Obraz':<45} | {'CLS':<3} | {'Z_calc':<7} | {'Z_gt':<7} | {'Błąd'}")
    print("-" * 85)

    errors = []

    for img_name in test_images:
        parts = img_name.replace(".png", "").split("_")
        scene_id, sensor_id, frame_idx = f"{parts[0]}_{parts[1]}", f"{parts[2]}_{parts[3]}", parts[4]

        frame = cv2.imread(os.path.join(image_dir, img_name))
        if frame is None: continue

        # System NURD
        _, risks = app.process_frame(frame.copy(), dt=0.1)
        
        # Ground Truth 3D
        gt_data = get_3d_gt(scene_id, sensor_id, frame_idx)
        if not gt_data: continue

        gt_objects = []
        for ann in gt_data.get('annotations', []):
            v_type = ann.get('attributes', {}).get('vehicle_type', '')
            cls = is_nurd(v_type)
            if cls != -1:
                # W CARLA Z to zazwyczaj odległość w głąb (radialna w kamerze)
                dist_gt = ann['box']['pose']['translation']['z']
                gt_objects.append({'cls': cls, 'z': abs(dist_gt)})

        # Matchowanie (uproszczone: po klasie i najbliższym Z)
        for r in risks:
            pred_z = r.get('distance_m', 0)
            pred_cls = 0 # YOLO class
            
            # Znajdź najbliższy GT o tej samej klasie
            best_match = None
            min_diff = float('inf')
            
            for gt in gt_objects:
                # Uwaga: klasy YOLO mogą się różnić, przyjmijmy że szukamy czegokolwiek NURD
                diff = abs(pred_z - gt['z'])
                if diff < min_diff and diff < 15.0: # Matchuj tylko jeśli błąd < 15m
                    min_diff = diff
                    best_match = gt
            
            if best_match:
                err = abs(pred_z - best_match['z'])
                errors.append(err)
                print(f"{img_name[:40]:<45} | {best_match['cls']:<3} | {pred_z:>6.2f}m | {best_match['z']:>6.2f}m | {err:>5.2f}m")

    if errors:
        print("-" * 85)
        print(f"Średni błąd bezwzględny (MAE): {np.mean(errors):.2f} metrów")
    else:
        print("\n[!] Nie dopasowano żadnych obiektów. Sprawdź klasy w JSON lub czy YOLO coś wykrywa.")

if __name__ == "__main__":
    validate_system()
