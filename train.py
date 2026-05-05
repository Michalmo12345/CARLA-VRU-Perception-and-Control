from ultralytics import YOLO

def main():

    model = YOLO('yolo11s.pt') 

    results = model.train(
        data='data.yaml',      # Path to your dataset config file
        epochs=3,            # How many times the model will see the whole dataset
        imgsz=640,             # Resize images to 640 pixels (maintains aspect ratio)
        batch=16,              # How many images to process at once (lower to 8 if you get Out Of Memory errors)
        device=0,              # '0' means use GPU. Change to 'cpu' if you don't have an NVIDIA GPU setup.
        workers=8,             # Number of CPU threads for data loading (speeds up training)
        project='TWM',         # Name of the main folder where results will be saved
        name='run_1'           # Name of this specific training run
    )

    print("Training complete!")

if __name__ == '__main__':
    main()