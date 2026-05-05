from ultralytics import YOLO

def main():

    model = YOLO('runs/detect/TWM/run_1-3/weights/best.pt') 


    results = model.predict(
        source='dataset/images/val', 
        conf=0.5,                    
        save=True,                  
        show=False,
        stream=True                
    )

if __name__ == '__main__':
    main()