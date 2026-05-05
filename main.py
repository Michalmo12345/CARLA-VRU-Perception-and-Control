from src.yolodatasetbuilder import YoloDatasetBuilder
from src.yolomanager import YoloManager

def main():
    # --- KONFIGURACJA ---
    # 'TRAIN' -> tylko trening (pomiń budowanie danych)
    # 'FULL'  -> buduj dane i trenuj
    # 'PREDICT' -> tylko detekcja
    MODE = 'VALIDATE' 

    DATA_YAML = 'data.yaml'  
    MODEL_BASE = 'models/yolo11s.pt'

    if MODE == 'FULL':
        print("\n[*] URUCHAMIANIE PIPELINE: Przygotowanie danych...")
        builder = YoloDatasetBuilder()
        builder.run()
    else:
        print("\n[*] POMINIĘTO: Etap przygotowania danych.")


    if MODE in ['TRAIN', 'FULL']:

        manager = YoloManager(model_path=MODEL_BASE, data_yaml=DATA_YAML)
        

        manager.train(
            epochs=100, 
            batch=16, 
            project_name='TWM',
            name='run_1' 
        )

    elif MODE == 'PREDICT':
        manager = YoloManager(model_path='runs/detect/TWM/run_1-3/weights/best.pt')
        manager.predict(
            source_path='dataset/images/val', 
            conf=0.5,                    
            save=True,                  
            show=False,
            stream=True                
        )

    elif MODE == 'VALIDATE':
        manager = YoloManager(model_path='runs/detect/TWM/run_1-3/weights/best.pt', data_yaml=DATA_YAML)
        manager.validate()
        
if __name__ == "__main__":
    main()