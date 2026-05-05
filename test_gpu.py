# Save this as test_gpu.py and run it
from ultralytics import YOLO
import torch

print(f"PyTorch CUDA available: {torch.cuda.is_available()}")
print(f"Number of GPUs found: {torch.cuda.device_count()}")
print(f"GPU Name: {torch.cuda.get_device_name(0)}")

model = YOLO("yolo11n.pt") 
print(f"Model device: {model.device}")