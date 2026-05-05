import os
import cv2
import random

CLASSES = ['pedestrian', 'cyclist', 'motorcyclist']
COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255)] # Kolory dla klas (B, G, R)

IMAGES_DIR = 'dataset/images/train'
LABELS_DIR = 'dataset/labels/train'
OUTPUT_DIR = 'verify_samples'

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    all_images = [f for f in os.listdir(IMAGES_DIR) if f.endswith('.png') or f.endswith('.jpg')]
    if not all_images:
        print("Nie znaleziono zdjęć w folderze train!")
        return
        
    sample_images = random.sample(all_images, min(5, len(all_images)))
    
    for img_filename in sample_images:
        img_path = os.path.join(IMAGES_DIR, img_filename)
        txt_filename = os.path.splitext(img_filename)[0] + '.txt'
        label_path = os.path.join(LABELS_DIR, txt_filename)
        
        img = cv2.imread(img_path)
        img_h, img_w, _ = img.shape

        if os.path.exists(label_path):
            with open(label_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) != 5: continue
                    
                    cls_id = int(parts[0])
                    x_center, y_center, w_norm, h_norm = map(float, parts[1:])
                    
                    # Odwrócenie normalizacji YOLO do pikseli (x, y, w, h)
                    w = int(w_norm * img_w)
                    h = int(h_norm * img_h)
                    x = int((x_center * img_w) - (w / 2))
                    y = int((y_center * img_h) - (h / 2))
                    
                    color = COLORS[cls_id % len(COLORS)]
                    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(img, CLASSES[cls_id], (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        out_path = os.path.join(OUTPUT_DIR, img_filename)
        cv2.imwrite(out_path, img)
        print(f"Zapisano próbkę: {out_path}")

if __name__ == '__main__':
    main()