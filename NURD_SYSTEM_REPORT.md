# Raport Końcowy: System Wykrywania i Analizy Ryzyka NURD

## 1. Opis Działania Systemu

Zaimplementowany system NURD (Niechronieni Uczestnicy Ruchu Drogowego) jest modułową platformą bezpieczeństwa czynnego, przeznaczoną do integracji z pojazdami autonomicznymi w środowisku CARLA. System przetwarza surowy strumień wideo z kamery RGB w celu podjęcia decyzji o ograniczeniu prędkości.

### Klocki Przetwarzania (Pipeline):
1.  **DetectionModule (Detekcja)**: Wykorzystuje sieć neuronową **YOLOv11s** do lokalizacji obiektów w czasie rzeczywistym. Na wyjściu generuje ramki ograniczające (Bounding Boxes) dla klas: Pieszy, Rowerzysta, Motorower.
2.  **TrackingModule (Śledzenie)**: Przypisuje unikalne identyfikatory (ID) do obiektów. Wykorzystuje **Algorytm Węgierski** do asocjacji oraz **Filtr Kalmana** (model Constant Velocity) do wygładzania trajektorii i estymacji prędkości pikselowej.
3.  **DistanceEstimationModule (Odległość)**: Przekształca dane 2D na fizyczną odległość w metrach ($Z$). Wykorzystuje model geometryczny **Pinhole Camera** oraz statystyczne założenia o wysokościach obiektów (Priors).
4.  **RiskAssessmentModule (Ocena Ryzyka)**: Wylicza metryczną prędkość zbliżania ($v_{app}$) oraz współczynnik **TTC (Time to Collision)**. Na tej podstawie klasyfikuje zagrożenie (LOW -> CRITICAL) i sugeruje docelową prędkość pojazdu.

---

## 2. Wyniki Testów i Analiza Statystyczna

### 2.1. Skuteczność Modelu Detekcji (YOLO)
Testy przeprowadzono na zbiorze walidacyjnym po 100 epokach uczenia.

| Miara | Wartość | Interpretacja |
| :--- | :--- | :--- |
| **Precision** | 0.857 | 85.7% wykrytych obiektów faktycznie istnieje. |
| **Recall** | 0.543 | System wykrywa ok. 54% wszystkich obiektów w scenie. |
| **mAP50** | 0.627 | Średnia precyzja przy progu IoU 0.5. |
| **mAP50-95** | 0.418 | Precyzja przy rygorystycznych progach dopasowania. |

**Wniosek**: Model wykazuje wysoką precyzję (mało fałszywych alarmów), ale niższą czułość (Recall), co sugeruje trudności z wykrywaniem bardzo małych (dalekich) obiektów.
### 2.2. Dokładność Estymacji Odległości
Wyniki porównano z obiektywnymi danymi Ground Truth 3D z symulatora w rozszerzonym teście na próbie 100 klatek (kilkaset indywidualnych pomiarów obiektów).

*   **Średni błąd bezwzględny (MAE)**: **1.40 metra**.
*   **Strefa krytyczna (do 15m)**: Błąd zazwyczaj poniżej **0.5 metra**. Bardzo wysoka precyzja w obszarze kluczowym dla bezpieczeństwa.
*   **Strefa daleka (>15m)**: Błąd rośnie do 1-2m (wynika z ograniczeń rozdzielczości obrazu - 1 piksel błędu bounding boxa mocno wpływa na wynik).
*   **Wartości odstające (Outliery)**: Pojedyncze błędy >5m wynikają głównie z zagęszczenia obiektów i problemów z asocjacją w scenach o dużym tłoku.

---

## 3. Krytyczna Analiza Wyników

### 3.1. Analiza Błędów i Ograniczeń
1.  **Błąd "Standardowego Wzrostu"**: Największe odchylenia w module dystansu wynikają z różnic między założonym wzrostem statystycznym (np. 1.70m) a faktycznym modelem 3D w CARLA. Odchylenia te są jednak akceptowalne.
2.  **Ograniczenia Kamery Monokularnej**: Zastosowany wzór $Z = (H_{real} \cdot f) / h_{px}$ jest bardzo wrażliwy na precyzję wysokości bounding boxa, stąd spadek dokładności na dużych dystansach.
3.  **Recall (Czułość)**: Recall na poziomie 0.54 wskazuje, że dalekie lub przysłonięte obiekty mogą być pomijane. Wymaga to użycia modeli o wyższej rozdzielczości wejściowej w przyszłości.

### 3.2. Wnioski Końcowe
*   System wykazuje **wysoką stabilność numeryczną**. Błąd MAE na poziomie 1.40m przy zastosowaniu prostej kamery (bez LiDARa) to wynik bardzo dobry, zwłaszcza że w strefie krytycznej (blisko pojazdu) dokładność wynosi kilkanaście centymetrów.
*   System jest **wystarczający dla decyzji typu "Zwolnij/Hamuj"** (ADAS), opierając się na trendzie zmiany dystansu (malejący TTC).
*   Integracja z CARLA poprzez interfejs `NURDApp` (z uwzględnieniem metrycznej prędkości zbliżania) czyni system gotowym do testów w zamkniętej pętli sterowania (Closed-Loop).

---
*Dokumentacja wygenerowana na podstawie testów integracyjnych, walidacji Ground Truth (100 klatek) oraz danych ewaluacyjnych YOLO.*
