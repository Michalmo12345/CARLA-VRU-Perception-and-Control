import os
import torch
from ultralytics import YOLO

class YoloManager:
    def __init__(self, model_path='yolo11s.pt', data_yaml='data.yaml'):
        """
        Inicjalizuje menedżera YOLO.
        :param model_path: Ścieżka do modelu .pt
        :param data_yaml: Ścieżka do pliku definicji danych
        """
        print(f"[*] Ładowanie modelu z: {model_path}...")
        self.model = YOLO(model_path)
        self.data_yaml = data_yaml
        
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        print(f"[*] Używane urządzenie: {self.device}")

    def train(self, epochs=100, batch=8, imgsz=640, project_name='TWM', name='run'):
        """Uruchamia proces uczenia."""
        if not os.path.exists(self.data_yaml):
            raise FileNotFoundError(f"Nie znaleziono pliku konfiguracji danych: {self.data_yaml}")

        print(f"[*] Rozpoczynam trening modelu na {epochs} epok...")
        self.model.train(
            data=self.data_yaml,
            epochs=epochs,
            imgsz=imgsz,
            batch=batch,
            device=self.device,
            project=project_name,
            name=name,
            exist_ok=True # Nadpisuje folder o tej samej nazwie zamiast tworzyć run2, run3...
        )
        print("[+] Trening zakończony!")

    def validate(self):
        """Ocenia skuteczność modelu (mAP)."""
        print("[*] Rozpoczynam walidację...")
        metrics = self.model.val(data=self.data_yaml)
        print("\n" + "="*30)
        print(f" Wyniki dla {self.data_yaml}:")
        print(f" - mAP50: {metrics.box.map50:.3f}")
        print(f" - mAP50-95: {metrics.box.map:.3f}")
        print("="*30)
        return metrics

    def predict(self, source_path, conf=0.5, save=True, show=False, stream=True):
        """Wykrywa obiekty na źródle (foto/wideo/stream)."""
        if not os.path.exists(source_path):
            print(f"[-] Błąd: Ścieżka {source_path} nie istnieje!")
            return

        print(f"[*] Detekcja: {source_path} (conf={conf})")
        results = self.model.predict(
            source=source_path,
            conf=conf,
            save=save,
            show=show,
            stream=stream
        )
        
        for r in results:
            pass
            
        print("[+] Przetwarzanie zakończone.")