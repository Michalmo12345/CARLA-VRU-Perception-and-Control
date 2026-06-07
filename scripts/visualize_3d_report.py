import sys, os; sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import os
import cv2
import numpy as np

def calculate_iou(box1, box2):
    """Oblicza Intersection over Union (IoU) dla dwóch ramek [x1, y1, x2, y2]."""
    xi1 = max(box1[0], box2[0])
    yi1 = max(box1[1], box2[1])
    xi2 = min(box1[2], box2[2])
    yi2 = min(box1[3], box2[3])
    
    inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
    
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    union_area = box1_area + box2_area - inter_area
    return inter_area / union_area if union_area > 0 else 0

def merge_boxes(boxes):
    """Łączy silnie nakładające się ramki w jedną."""
    if not boxes: return []
    
    merged = []
    used = set()
    
    for i in range(len(boxes)):
        if i in used: continue
        
        current_box = list(boxes[i])
        used.add(i)
        
        # Szukamy innych ramek do złączenia
        for j in range(i + 1, len(boxes)):
            if j in used: continue
            
            # Jeśli ramki nakładają się mocno (IoU > 0.1 - w przypadku motocyklisty wystarczy styk)
            if calculate_iou(current_box[1:], boxes[j][1:]) > 0.05:
                # Rozszerzamy current_box tak, aby obejmował obie ramki
                current_box[1] = min(current_box[1], boxes[j][1]) # x1
                current_box[2] = min(current_box[2], boxes[j][2]) # y1
                current_box[3] = max(current_box[3], boxes[j][3]) # x2
                current_box[4] = max(current_box[4], boxes[j][4]) # y2
                used.add(j)
        
        merged.append(current_box)
    
    return merged

def main():
    image_dir = "dataset/images/train"
    label_dir = "dataset/labels/train"
    out_dir = "report_visuals"
    os.makedirs(out_dir, exist_ok=True)
    
    target_w, target_h = 2048, 1024
    test_images = [f for f in os.listdir(image_dir) if f.endswith(".png")][:15]
    
    for img_name in test_images:
        img = cv2.imread(os.path.join(image_dir, img_name))
        if img is None: continue
        img = cv2.resize(img, (target_w, target_h))
        
        label_path = os.path.join(label_dir, img_name.replace(".png", ".txt"))
        
        if os.path.exists(label_path):
            raw_boxes = []
            with open(label_path, 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) < 5: continue
                    cls, cx, cy, bw, bh = map(float, parts)
                    # Konwersja do XYXY
                    x1, y1 = (cx - bw/2), (cy - bh/2)
                    x2, y2 = (cx + bw/2), (cy + bh/2)
                    raw_boxes.append([int(cls), x1, y1, x2, y2])
            
            # Grupowanie i łączenie ramek (Virtual Merge)
            final_boxes = []
            for c in range(3): # Dla każdej klasy NURD
                class_boxes = [b for b in raw_boxes if b[0] == c]
                final_boxes.extend(merge_boxes(class_boxes))
            
            # Rysowanie połączonych ramek
            for box in final_boxes:
                cls, x1, y1, x2, y2 = box
                px1, py1 = int(x1 * target_w), int(y1 * target_h)
                px2, py2 = int(x2 * target_w), int(y2 * target_h)
                
                color = (255, 255, 255)
                if cls == 1: color = (0, 255, 255)
                if cls == 2: color = (0, 165, 255)
                
                cv2.rectangle(img, (px1, py1), (px2, py2), color, 2)
                class_name = ['Pedestrian', 'Cyclist', 'Motorcyclist'][cls]
                cv2.putText(img, class_name, (px1, py1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.putText(img, "CONSOLIDATED GROUND TRUTH (MERGED)", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        out_path = os.path.join(out_dir, f"gt_val_{img_name}")
        cv2.imwrite(out_path, img)
        print(f"Wygenerowano (Merged): {out_path}")

if __name__ == "__main__":
    main()
