import os
import json
import random
import shutil

CLASS_MAPPING = {
    22: 0,  # Pieszy
    1: 1, 2: 1,   # Rower i Rowerzysta -> Klasa 1
    13: 2, 14: 2  # Motocykl i Motocyklista -> Klasa 2
}

IMAGE_WIDTH = 2048 
IMAGE_HEIGHT = 1024

ORIGINAL_JSON_DIR = 'bounding_box_2d'
ORIGINAL_IMAGES_DIR = 'rgb'
YOLO_DATASET_DIR = 'dataset' 

SPLIT_RATIO = 0.8 
IMAGE_EXT = '.png' 

def convert_box_to_yolo(x, y, w, h, img_width, img_height):
    x_c = (x + w / 2) / img_width
    y_c = (y + h / 2) / img_height
    w_n = w / img_width
    h_n = h / img_height
    return x_c, y_c, w_n, h_n

def get_all_files(directory, extension=None):
    file_paths = []
    for root, _, files in os.walk(directory):
        for file in files:
            if extension is None or file.endswith(extension):
                file_paths.append(os.path.join(root, file))
    return file_paths

def process_split(file_list, split_name):
    print(f"Przetwarzanie zbioru: {split_name} ({len(file_list)} plików)...")
    
    # Tworzenie wymaganych przez YOLO folderów (np. dataset/images/train, dataset/labels/train)
    labels_dir = os.path.join(YOLO_DATASET_DIR, 'labels', split_name)
    images_dir = os.path.join(YOLO_DATASET_DIR, 'images', split_name)
    os.makedirs(labels_dir, exist_ok=True)
    os.makedirs(images_dir, exist_ok=True)

    for json_path in file_list:
        # 1. Odczytanie struktury folderów (np. scene_xxxxxx/CS_FRONT)
        rel_path = os.path.relpath(json_path, ORIGINAL_JSON_DIR)
        dir_name = os.path.dirname(rel_path)
        json_filename = os.path.basename(rel_path) # xxxxxxxx_yyyyyyyyyyyyy.json
        
        # 2. Mapowanie nazwy: bierzemy 'xxxxxxxx' z 'xxxxxxxx_yyyyyyyyyyyyy.json'
        base_name_parts = json_filename.replace('.json', '').split('_')
        original_base_name = base_name_parts[0] 
        
        # 3. Odnalezienie odpowiadającego obrazu w folderze rgb/
        image_path = os.path.join(ORIGINAL_IMAGES_DIR, dir_name, original_base_name + IMAGE_EXT)
        
        if not os.path.exists(image_path):
            print(f"Ostrzeżenie: Nie znaleziono obrazu: {image_path}")
            continue

        # 4. Spłaszczanie nazw (zabezpieczenie)
        # Zmieniamy scene_01/CS_FRONT/0000.png na scene_01_CS_FRONT_0000.png
        # Pozwala to trzymać wszystkie zdjęcia w jednym płaskim folderze YOLO bez ryzyka nadpisania
        safe_prefix = dir_name.replace(os.sep, '_').replace('/', '_').replace('\\', '_')
        yolo_base_name = f"{safe_prefix}_{original_base_name}"
        
        dest_txt_path = os.path.join(labels_dir, yolo_base_name + '.txt')
        dest_img_path = os.path.join(images_dir, yolo_base_name + IMAGE_EXT)

        # 5. Konwersja koordynat do YOLO
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        yolo_lines = []
        for ann in data.get('annotations', []):
            original_class_id = ann.get('class_id')
            
            if original_class_id in CLASS_MAPPING:
                visibility = ann.get('attributes', {}).get('visibility', 1.0)
                if visibility < 0.3:
                    continue
                    
                yolo_class_id = CLASS_MAPPING[original_class_id]
                box = ann.get('box', {})
                x, y, w, h = box.get('x'), box.get('y'), box.get('w'), box.get('h')
                
                if None not in (x, y, w, h):
                    x_c, y_c, w_n, h_n = convert_box_to_yolo(x, y, w, h, IMAGE_WIDTH, IMAGE_HEIGHT)
                    yolo_lines.append(f"{yolo_class_id} {x_c:.6f} {y_c:.6f} {w_n:.6f} {h_n:.6f}")
        
        # 6. Zapis nowej etykiety i kopiowanie zdjęcia (tylko jeśli wszystko jest OK)
        with open(dest_txt_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(yolo_lines))
            
        shutil.copy(image_path, dest_img_path)

def main():
    all_jsons = get_all_files(ORIGINAL_JSON_DIR, extension='.json')
    
    if not all_jsons:
        print("Nie znaleziono plików JSON we wskazanym folderze!")
        return

    random.seed(42) 
    random.shuffle(all_jsons)

    split_idx = int(len(all_jsons) * SPLIT_RATIO)
    
    train_files = all_jsons[:split_idx]
    val_files = all_jsons[split_idx:]

    print(f"Znaleziono {len(all_jsons)} adnotacji.")
    print(f"Do treningu trafi: {len(train_files)}")
    print(f"Do walidacji (testów) trafi: {len(val_files)}\n")

    # Przetwarzamy oba sety danych jedną dedykowaną funkcją
    process_split(train_files, 'train')
    process_split(val_files, 'val')

    print("\nKonwersja i podział zakończone pomyślnie!")

if __name__ == "__main__":
    main()