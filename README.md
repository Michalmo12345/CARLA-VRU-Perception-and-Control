# TWM YOLO Dataset Builder & Manager

This project is a comprehensive pipeline for building YOLO-compatible datasets and managing the training/validation/inference lifecycle of YOLO models (specifically tested with YOLO11).

## Prerequisites

- Python 3.8+
- NVIDIA GPU with CUDA support (recommended for training)

## Setup Instructions

### 1. Get the Data

This project is designed to work with data from **[RealDriveSim](https://realdrivesim.github.io/)**. To create the dataset, you need the raw images and annotations provided by the simulator.

Ensure you have the following directories in the project root:
- `rgb/`: Images from the simulator.
- `bounding_box_2d/`: JSON annotations.

*(Note: You can configure these paths in `src/yolodatasetbuilder/config.py`)*

### 2. Environment Setup

It is recommended to use a virtual environment:

#### Using venv:
```bash
python -m venv yolo_env
source yolo_env/bin/activate  # On Linux/macOS
# yolo_env\Scripts\activate  # On Windows
```

#### Install Requirements:
```bash
pip install -r requirements.txt
```

## Usage

The main entry point is `main.py`. It supports different modes via the `-mode` (or `-model`) flag.

### Prepare Data and Train (FULL Mode)
To build the dataset from raw files and immediately start training:
```bash
python main.py -model FULL
```

### Other Modes
- **TRAIN**: Skip data building and start training on existing dataset.
  ```bash
  python main.py -mode TRAIN
  ```
- **VALIDATE**: Run validation on the trained model.
  ```bash
  python main.py -mode VALIDATE
  ```
- **PREDICT**: Run inference using the trained model.
  ```bash
  python main.py -mode PREDICT
  ```

## Project Structure

- `src/yolodatasetbuilder/`: Logic for converting raw JSON annotations to YOLO format.
- `src/yolomanager/`: Wrapper for Ultralytics YOLO training and inference.
- `dataset/`: Generated YOLO dataset (images and labels).
- `data.yaml`: Configuration for the YOLO dataset (classes, paths).
- `runs/`: Output directory for training logs and weights.

## Configuration

You can modify `src/yolodatasetbuilder/config.py` to change:
- Class mappings
- Image dimensions
- Train/Val split ratio
- Directory paths
