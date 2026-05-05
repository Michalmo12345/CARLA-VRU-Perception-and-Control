import os
import shutil

def get_all_files(directory, extension=None):
    """Recursively fetches all files with a specific extension."""
    file_paths = []
    for root, _, files in os.walk(directory):
        for file in files:
            if extension is None or file.endswith(extension):
                file_paths.append(os.path.join(root, file))
    return file_paths

def create_yolo_directories(base_dir, split_name):
    """Creates the necessary YOLO folder structure."""
    labels_dir = os.path.join(base_dir, 'labels', split_name)
    images_dir = os.path.join(base_dir, 'images', split_name)
    os.makedirs(labels_dir, exist_ok=True)
    os.makedirs(images_dir, exist_ok=True)
    return images_dir, labels_dir

def copy_file(src, dst):
    shutil.copy(src, dst)

def save_text_file(lines, dst_path):
    with open(dst_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))