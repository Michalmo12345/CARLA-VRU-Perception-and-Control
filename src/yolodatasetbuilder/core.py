import os
import json
import random
from . import config
from . import geometry
from . import files

class YoloDatasetBuilder:
    def __init__(self):
        self.jsons = files.get_all_files(config.ORIGINAL_JSON_DIR, extension='.json')

    def run(self):
        if not self.jsons:
            print("No JSON files found in the specified directory!")
            return

        random.seed(config.RANDOM_SEED)
        random.shuffle(self.jsons)
        
        split_idx = int(len(self.jsons) * config.SPLIT_RATIO)
        train_files = self.jsons[:split_idx]
        val_files = self.jsons[split_idx:]

        print(f"Found {len(self.jsons)} annotations.")
        print(f"Training: {len(train_files)} | Validation: {len(val_files)}\n")

        self._process_split(train_files, 'train')
        self._process_split(val_files, 'val')
        
        print("\nDataset successfully built and structured!")

    def _process_split(self, file_list, split_name):
        print(f"Processing {split_name} split...")
        images_dir, labels_dir = files.create_yolo_directories(config.YOLO_DATASET_DIR, split_name)

        for json_path in file_list:
            rel_path = os.path.relpath(json_path, config.ORIGINAL_JSON_DIR)
            dir_name = os.path.dirname(rel_path)
            json_filename = os.path.basename(rel_path)

            base_name_parts = json_filename.replace('.json', '').split('_')
            original_base_name = base_name_parts[0] 
            
            image_path = os.path.join(config.ORIGINAL_IMAGES_DIR, dir_name, original_base_name + config.IMAGE_EXT)
            
            if not os.path.exists(image_path):
                print(f"Warning: Missing image -> {image_path}")
                continue

            safe_prefix = dir_name.replace(os.sep, '_').replace('/', '_').replace('\\', '_')
            yolo_base_name = f"{safe_prefix}_{original_base_name}"
            
            dest_txt_path = os.path.join(labels_dir, yolo_base_name + '.txt')
            dest_img_path = os.path.join(images_dir, yolo_base_name + config.IMAGE_EXT)

            yolo_lines = self._parse_json(json_path)
            files.save_text_file(yolo_lines, dest_txt_path)
            files.copy_file(image_path, dest_img_path)

    def _parse_json(self, json_path):
        """Reads a single JSON and returns a list of YOLO formatted strings."""
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        yolo_lines = []
        for ann in data.get('annotations', []):
            class_id = ann.get('class_id')
            
            if class_id in config.CLASS_MAPPING:
                visibility = ann.get('attributes', {}).get('visibility', 1.0)
                if visibility < config.MIN_VISIBILITY:
                    continue
                    
                yolo_cls = config.CLASS_MAPPING[class_id]
                box = ann.get('box', {})
                x, y, w, h = box.get('x'), box.get('y'), box.get('w'), box.get('h')
                
                if None not in (x, y, w, h):
                    x_c, y_c, w_n, h_n = geometry.convert_box_to_yolo(
                        x, y, w, h, config.IMAGE_WIDTH, config.IMAGE_HEIGHT
                    )
                    yolo_lines.append(f"{yolo_cls} {x_c:.6f} {y_c:.6f} {w_n:.6f} {h_n:.6f}")
                    
        return yolo_lines