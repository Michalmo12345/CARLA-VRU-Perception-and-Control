import argparse
from src.yolodatasetbuilder import YoloDatasetBuilder
from src.yolomanager import YoloManager

def main():
    parser = argparse.ArgumentParser(description="TWM YOLO Pipeline")
    parser.add_argument(
        "-mode", "--mode", 
        choices=["FULL", "TRAIN", "PREDICT", "VALIDATE"], 
        default="VALIDATE",
        help="Pipeline mode: FULL (build + train), TRAIN (train only), PREDICT (inference), VALIDATE (metrics)"
    )
    # Adding -model as requested by user, treating it as mode
    parser.add_argument("-model", dest="mode_alias", help="Alias for -mode")
    
    args = parser.parse_args()
    
    MODE = args.mode_alias if args.mode_alias else args.mode

    DATA_YAML = 'data.yaml'  
    MODEL_BASE = 'models/yolo11s.pt'

    if MODE == 'FULL':
        print(f"\n[*] URUCHAMIANIE PIPELINE: Przygotowanie danych (Mode: {MODE})...")
        builder = YoloDatasetBuilder()
        builder.run()
    else:
        print(f"\n[*] POMINIĘTO: Etap przygotowania danych (Mode: {MODE}).")


    if MODE in ['TRAIN', 'FULL']:
        manager = YoloManager(model_path=MODEL_BASE, data_yaml=DATA_YAML)
        manager.train(
            epochs=100, 
            batch=16, 
            project_name='TWM',
            name='run_1' 
        )

    elif MODE == 'PREDICT':
        manager = YoloManager(model_path='runs/detect/TWM/run/weights/best.pt')
        manager.predict(
            source_path='dataset/images/val', 
            conf=0.5,                    
            save=True,                  
            show=False,
            stream=True                
        )

    elif MODE == 'VALIDATE':
        manager = YoloManager(model_path='runs/detect/TWM/run/weights/best.pt', data_yaml=DATA_YAML)
        manager.validate()
        
if __name__ == "__main__":
    main()