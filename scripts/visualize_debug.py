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

def main():
    weights = 'runs/detect/TWM/run/weights/best.pt'
    image_dir = "dataset/images/train"
    out_dir = "report_visuals"
    os.makedirs(out_dir, exist_ok=True)
    
    # 1. Grupowanie obrazów po scenach
    all_imgs = [f for f in os.listdir(image_dir) if f.endswith(".png")]
    scenes = {}
    for img in all_imgs:
        parts = img.replace(".png", "").split("_")
        scene_key = f"{parts[0]}_{parts[1]}_{parts[2]}_{parts[3]}" # scene_000000_CS_FRONT
        if scene_key not in scenes: scenes[scene_key] = []
        scenes[scene_key].append(img)
    
    # Wybieramy 15 scen, które mają oba wymagane kadry (25 i 75)
    valid_scenes = []
    for skey, imgs in scenes.items():
        if any("00025" in i for i in imgs) and any("00075" in i for i in imgs):
            valid_scenes.append(skey)
    
    np.random.shuffle(valid_scenes)
    selected_scenes = valid_scenes[:15]

    print(f"[*] Rozpoczynam generowanie kinemtyki dla {len(selected_scenes)} scen...")

    for skey in selected_scenes:
        # Nowa instancja aplikacji dla każdej sceny (RESET trackera)
        app = NURDApp(weights)
        
        # Sortujemy obrazki, aby klatka 25 była pierwsza
        scene_imgs = sorted([i for i in scenes[skey] if "00025" in i or "00075" in i])
        
        # Przetwarzamy klatki po kolei
        for i, img_name in enumerate(scene_imgs):
            parts = img_name.replace(".png", "").split("_")
            scene_id, sensor_id, frame_idx = f"{parts[0]}_{parts[1]}", f"{parts[2]}_{parts[3]}", parts[4]
            
            frame = cv2.imread(os.path.join(image_dir, img_name))
            if frame is None: continue
            
            # dt = 0.5s (bo między klatką 25 a 75 w CARLA zazwyczaj mija ok. 0.5s przy 10Hz)
            # Dla pierwszej klatki dt nie ma znaczenia (inicjalizacja), dla drugiej kluczowe
            processed_frame, risks = app.process_frame(frame, dt=0.5)
            
            # Zapisujemy tylko drugą klatkę (indeks 75), bo tam już widać wektor prędkości
            if "00075" in img_name:
                gt_data = get_3d_gt(scene_id, sensor_id, frame_idx)
                if gt_data:
                    gt_dists = []
                    for ann in gt_data.get('annotations', []):
                        if is_nurd(ann.get('attributes', {}).get('vehicle_type', '')) != -1:
                            gt_dists.append(abs(ann['box']['pose']['translation']['z']))
                    
                    # Rysowanie panelu walidacji na obrazku
                    for j, r in enumerate(risks):
                        pred_z = r.get('distance_m', 0)
                        if gt_dists:
                            closest_gt = min(gt_dists, key=lambda x: abs(x - pred_z))
                            error = abs(pred_z - closest_gt)
                            info = f"ID:{int(r['track_id'])} CALC:{pred_z:.1f}m | GT:{closest_gt:.1f}m | ERR:{error:.1f}m"
                            cv2.putText(processed_frame, info, (20, 70 + j*30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                cv2.putText(processed_frame, f"SCENE: {skey} (FRAME 0075)", (20, 35), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                out_path = os.path.join(out_dir, f"kinematic_val_{img_name}")
                cv2.imwrite(out_path, processed_frame)
                print(f"Wygenerowano dowód kinematyki: {out_path}")

if __name__ == "__main__":
    main()
