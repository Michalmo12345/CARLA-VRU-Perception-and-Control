# System wykrywania niechronionych uczestników ruchu drogowego

Michał Mokrzycki<br>Antoni Grajek<br>Tadeusz Chmielik

Wynikiem projektu jest system działający w symulatorze CARLA. Rozwiązanie przetwarza dane pochodzący z kamery symulowanego samochodu, na tej podstawie system określa położenie oraz rodzaj NURD (niechronionego uczestnika ruchu drogowego: pieszy, rowerzysta, motorower). W kolejnym kroku określana jest prędkość oraz kierunek tego osobnika. Na końcu system decyduje o zmianie prędkości pojazdu sterowanego.

Kluczowym wymogiem rozwiązania będzie jego odpornosć na warunki pogodowe, oświetlenie oraz natężenie ruchu drogowego.

## Przegląd istniejących rozwiązań wraz ze wskazaniem ich mocnych i słabych stron

- Klasyczne detektory (YOLO, Faster R-CNN, CenterNet)<br>Opis: Detekcja pieszych na obrazach RGB z użyciem modeli deep learning.<br>Mocne strony: wysoka dokładność, możliwość działania w czasie rzeczywistym, łatwa implementacja.<br>Słabe strony: duże wymagania danych, wrażliwość na warunki, brak informacji o głębi, problem domain gap.

- Ulepszone modele (attention, multi-scale, modyfikacje YOLO)<br>Opis: Rozszerzone architektury poprawiające wykrywanie małych i zasłoniętych obiektów.<br>Mocne strony: wyższa skuteczność, lepszy recall, nadal szybkie działanie.<br>Słabe strony: większa złożoność, trudniejsze trenowanie, nadal problem domain gap.

- Podejścia wielosensorowe (kamera + LiDAR)<br>Opis: Fuzja danych z wielu sensorów (2D + 3D).<br>Mocne strony: większa niezawodność, odporność na warunki, dostęp do głębi.<br>Słabe strony: wysoka złożoność, większe wymagania sprzętowe, trudna integracja.

Omówienie zbioru danych, który zostanie wykorzystany do treningu i testów:

- RealDriveSim: [https://realdrivesim.github.io/](https://realdrivesim.github.io/)<br>Nowoczesny, wielomodalny i wielozadaniowy zbiór syntetyczny wygenerowany w symulatorze CARLA. Charakteryzuje sie on najwyższym stopniem realizmu wirtualnego. Ponadto z racji tego, że zbiór danych wygenerowany jest z samego symulatora, można testować w żywym symulatorze

- Ponad 130 000 klatek danych w ponad 6500 sekwencjach zebrane na 10 różnych mapach (miasto, środowiska podmiejskie, autostrady).

- RealDriveSim dostarcza zsynchronizowane obrazy z kamery przedniej oraz chmury punktów z LiDAR-ów 32- i 64-kanałowych.

- Adnotacje dla 64 różnych klas obiektów

- Szczegółowe rozróżnienie przechodniów rowerzystów czy innych pojazdów.

- Zbiór ten zostanie wykorzystany do trenowania modeli detekcji 2D/3D a LiDAR pozwala na precyzyjną estymacje odległości

Przykładowe zdjęcia:

![](img/image11.png)

![](img/image10.png)

Format jsona dla etykietowanych danych:

![](img/image6.png)

Projekt techniczny rozwiązania (np. schemat struktury systemu ze wskazaniem najważniejszych bloków obliczeniowych, używanych w nich algorytmów oraz rodzaju danych przekazywanych pomiędzy blokami).

Projektowany system będzie działał w środowisku symulatora CARLA i będzie oparty na przetwarzaniu obrazu z przedniej kamery pojazdu. Celem rozwiązania jest wykrywanie niechronionych uczestników ruchu drogowego, określanie ich położenia, kierunku ruchu oraz prędkości, a następnie dostosowanie prędkości pojazdu autonomicznego w zależności od poziomu zagrożenia.

Architektura systemu będzie składać się z kilku głównych bloków obliczeniowych.

Pierwszym z nich będzie moduł akwizycji danych, odpowiedzialny za pobieranie obrazu RGB z kamery umieszczonej w symulowanym pojeździe. Obraz ten będzie przekazywany do modułu wstępnego przetwarzania, gdzie zostanie poddany operacjom takim jak skalowanie, normalizacja oraz ewentualna korekcja jasności i kontrastu. Celem tego etapu jest ujednolicenie danych wejściowych i zwiększenie odporności systemu na zmienne warunki oświetleniowe oraz pogodowe.

Następnie obraz trafi do modułu detekcji obiektów, którego zadaniem będzie wykrywanie i klasyfikowanie NURD. W tym bloku planowane jest zastosowanie modelu z rodziny YOLO, ze względu na jego wysoką szybkość działania oraz możliwość pracy w czasie rzeczywistym. Wyjściem modułu będą prostokąty ograniczające obiekty, przypisane klasy oraz poziomy pewności detekcji. System będzie rozróżniał trzy podstawowe klasy: pieszych, rowerzystów oraz motorowery.

Kolejnym elementem będzie moduł śledzenia obiektów w kolejnych klatkach obrazu. Jego zadaniem będzie przypisanie identyfikatorów wykrytym uczestnikom ruchu i obserwacja zmian ich położenia w czasie. Dzięki temu możliwe będzie wyznaczenie trajektorii ruchu każdego obiektu. Na podstawie zmian położenia między kolejnymi klatkami system będzie estymował kierunek poruszania się oraz prędkość obiektu. W tym celu można wykorzystać algorytmy śledzenia wieloobiektowego, takie jak ByteTrack lub Deep SORT, wspierane przez filtrację wygładzającą, na przykład filtr Kalmana.

Istotnym blokiem systemu będzie także moduł estymacji odległości. Ponieważ dane wejściowe podczas działania systemu będą pochodziły z kamery, odległość obiektu od pojazdu będzie wyznaczana na podstawie analizy obrazu. Jednocześnie zbiór RealDriveSim, zawierający dodatkowo dane LiDAR oraz mapy głębi, zostanie wykorzystany na etapie trenowania i testowania modelu, aby poprawić dokładność estymacji położenia obiektów. Pozwoli to połączyć zalety systemu wizyjnego działającego online z dodatkowymi informacjami przestrzennymi dostępnymi podczas przygotowania rozwiązania.

Na podstawie informacji o klasie obiektu, jego położeniu, prędkości oraz kierunku ruchu działać będzie moduł oceny ryzyka. Jego zadaniem będzie określenie, czy wykryty uczestnik ruchu stanowi zagrożenie dla toru jazdy pojazdu. W przypadku stwierdzenia zagrożenia system wyznaczy odpowiednią reakcję, polegającą na zmniejszeniu prędkości pojazdu. W najprostszym wariancie decyzja ta może być oparta na progach odległości i przewidywanym czasie do potencjalnej kolizji. Dla obiektów znajdujących się daleko od toru jazdy pojazd utrzyma zadaną prędkość, dla obiektów zbliżających się do pasa ruchu nastąpi częściowe ograniczenie prędkości, natomiast w sytuacji wysokiego ryzyka zostanie wygenerowana komenda silnego hamowania.

Końcowym elementem architektury będzie moduł sterowania pojazdem w symulatorze CARLA. Odbierze on decyzję z modułu oceny ryzyka i przekształci ją w sygnały sterujące, takie jak wartość przyspieszenia lub hamowania. W ten sposób system będzie tworzył pełny łańcuch działania: od obserwacji sceny drogowej, przez analizę obrazu i ocenę zachowania uczestników ruchu, aż po reakcję pojazdu autonomicznego.

![](img/image2.png)

### 
## Detekcja pieszych i rowerzystów
Stworzono system detekcji piszeych i roworzestów na obrazach z symulatora Carla. 

### Macierz pomyłek:

Macierz pomyłek potwierdza wysoką precyzję dla klasy Pedestrian oraz Cyclist. Przekątna macierzy

(wartości znormalizowane) wskazuje, że większość błędów to 'Background FP' (tło uznane za obiekt) lub pominięcia małych

obiektów, a nie błędne mylenie pieszego z rowerzystą.

![](img/image3.png)

### Training metrics:

Wykresy pokazują spadek funkcji kosztu (Box Loss, Class Loss) na zbiorze treningowym i walidacyjnym,

co świadczy o prawidłowym procesie uczenia bez zjawiska przeuczenia (overfittingu). Widać również stabilny wzrost mAP50 i

Precision, które stabilizują się po około 80 epokach.<br><br>Testy na 100 epochach.

![](img/image1.png)

### Krzywa Precision-Recall:

Krzywa Precision-Recall obrazuje kompromis między wykrywalnością a liczbą fałszywych alarmów. Pole

pod krzywą (AUC) dla wszystkich klas NURD wynosi 0.627, co potwierdza solidne zdolności detekcyjne modelu w zróżnicowanych

warunkach oświetleniowych symulatora.

![](img/image4.png)

### Przykładowe detekcje 2D

Przykładowe wyniki detekcji na danych testowych pokazują, że model poprawnie lokalizuje obiekty NURD

nawet przy ich znacznym zagęszczeniu (np. grupy pieszych). Ramki są ciasno dopasowane do sylwetek, co jest kluczowe dla

późniejszej estymacji odległości metodą geometryczną.

![](img/image5.png)

Połączone ramki motocykla z motocyklistą i test nocą:

![](img/image9.png)

Przykładowe nałożenie Bounding box3D i porównanie z obliczonymi wektorami<br>CALC to te obliczone, ERR to błąd

![](img/image7.png)

![](img/image8.png)




















##  Moduł Śledzenia 

Śledzenie obiektów opiera się na dwóch głównych filarach: asocjacji danych (Data Association) oraz filtracji trajektorii za pomocą Filtra Kalmana.

### Filtr Kalmana (Model Stałej Prędkości)
Zastosowano dyskretny Filtr Kalmana (Linear Kalman Filter) do estymacji stanu obiektów w 2D. 

*   **Wektor stanu ($\mathbf{x}$)**: 
    $$ \mathbf{x}_k = \begin{bmatrix} c_x \\ c_y \\ v_x \\ v_y \end{bmatrix} $$
    Gdzie $(c_x, c_y)$ to współrzędne środka obiektu, a $(v_x, v_y)$ to jego prędkość chwilowa w pikselach/s.

*   **Macierz przejścia stanu ($\mathbf{F}$)**:
    Definiuje dynamikę systemu (ruch jednostajny prostoliniowy między klatkami):
    $$ \mathbf{F} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} $$

*   **Macierz pomiaru ($\mathbf{H}$)**:
    Mapuje stan na przestrzeń pomiarową (YOLO dostarcza tylko pozycję, bez prędkości):
    $$ \mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix} $$

*   **Macierz kowariancji szumu procesu ($\mathbf{Q}$)**:
    Reprezentuje niepewność modelu (nagłe zmiany kierunku lub prędkości):
    $$ \mathbf{Q} = \sigma_q^2 \cdot \mathbf{I}_4 = \begin{bmatrix} \sigma_q^2 & 0 & 0 & 0 \\ 0 & \sigma_q^2 & 0 & 0 \\ 0 & 0 & \sigma_q^2 & 0 \\ 0 & 0 & 0 & \sigma_q^2 \end{bmatrix} $$

*   **Macierz kowariancji szumu pomiaru ($\mathbf{R}$)**:
    Reprezentuje błąd sensora (niedokładność bounding boxów YOLO):
    $$ \mathbf{R} = \sigma_r^2 \cdot \mathbf{I}_2 = \begin{bmatrix} \sigma_r^2 & 0 \\ 0 & \sigma_r^2 \end{bmatrix} $$

#### A. Faza Predykcji (Time Update)
1.  **Predykcja stanu**: $\mathbf{\hat{x}}_{k|k-1} = \mathbf{F} \cdot \mathbf{\hat{x}}_{k-1|k-1}$
2.  **Predykcja kowariancji błędu**: $\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1|k-1} \mathbf{F}^T + \mathbf{Q}$

#### B. Faza Korekty (Measurement Update)
1.  **Obliczenie innowacji**: $\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \mathbf{\hat{x}}_{k|k-1}$
2.  **Wzmocnienie Kalmana**: $\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T (\mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R})^{-1}$
3.  **Aktualizacja stanu**: $\mathbf{\hat{x}}_{k|k} = \mathbf{\hat{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$
4.  **Aktualizacja kowariancji**: $\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$

---

## Moduł Estymacji Odległości (Distance Estimation)

Wykorzystuje model kamery otworkowej (**Pinhole Camera Model**) do przejścia z 2D do 3D.

*   **Wzór główny**:
    $$ Z = \frac{H_{real} \cdot f}{h_{pixel}} $$

    Gdzie:
    *   $Z$ - Odległość wzdłuż osi optycznej [metry].
    *   $H_{real}$ - Fizyczna wysokość obiektu [metry] (Priors: Pieszy 1.7m, Rowerzysta 1.65m, Motorower 1.5m).
    *   $f$ - Ogniskowa kamery [piksele].
    *   $h_{pixel}$ - Wysokość ramki na obrazie [piksele].

---

## Moduł Oceny Ryzyka i Kinematyki Przestrzennej

### Prędkość Zbliżania (Radial Approach Velocity)
Wyliczana jako pochodna dystansu fizycznego $Z$ w czasie:
$$ v_{app} = \frac{Z_{t - \Delta t} - Z_{t}}{\Delta t} $$

### Czas do Kolizji (Time To Collision - TTC)
Określa czas do zderzenia przy założeniu stałej prędkości zbliżania:
$$ TTC = \frac{Z_t}{v_{app}} $$
*(Dla $v_{app} \le 0$ przyjmuje się $TTC = \infty$)*.

### Logika Decyzyjna
*   **CRITICAL**: $TTC < 1.5\text{s}$ lub $Z < 5\text{m}$ $\rightarrow$ Hamowanie awaryjne ($V_{target} = 0$).
*   **HIGH**: $TTC < 4.0\text{s}$ $\rightarrow$ Mocne zwolnienie ($V_{target} = 30\% \cdot V_{base}$).
*   **MEDIUM**: $Z < 20\text{m}$ $\rightarrow$ Lekkie zwolnienie ($V_{target} = 70\% \cdot V_{base}$).
*   **LOW**: Pozostałe $\rightarrow$ Utrzymanie prędkości ($V_{target} = V_{base}$).















## System Wykrywania i Analizy Ryzyka NURD

Zaimplementowany system NURD (Niechronieni Uczestnicy Ruchu Drogowego) jest modułową platformą bezpieczeństwa czynnego, przeznaczoną do integracji z pojazdami autonomicznymi w środowisku CARLA. System przetwarza surowy strumień wideo z kamery RGB w celu podjęcia decyzji o ograniczeniu prędkości.

### Klocki Przetwarzania (Pipeline):
1.  **DetectionModule (Detekcja)**: Wykorzystuje sieć neuronową **YOLOv11s** do lokalizacji obiektów w czasie rzeczywistym. Na wyjściu generuje ramki ograniczające (Bounding Boxes) dla klas: Pieszy, Rowerzysta, Motorower.
2.  **TrackingModule (Śledzenie)**: Przypisuje unikalne identyfikatory (ID) do obiektów. Wykorzystuje **Algorytm Węgierski** do asocjacji oraz **Filtr Kalmana** (model Constant Velocity) do wygładzania trajektorii i estymacji prędkości pikselowej.
3.  **DistanceEstimationModule (Odległość)**: Przekształca dane 2D na fizyczną odległość w metrach ($Z$). Wykorzystuje model geometryczny **Pinhole Camera** oraz statystyczne założenia o wysokościach obiektów (Priors).
4.  **RiskAssessmentModule (Ocena Ryzyka)**: Wylicza metryczną prędkość zbliżania ($v_{app}$) oraz współczynnik **TTC (Time to Collision)**. Na tej podstawie klasyfikuje zagrożenie (LOW -> CRITICAL) i sugeruje docelową prędkość pojazdu.

---



### Skuteczność Modelu Detekcji (YOLO)
Testy przeprowadzono na zbiorze walidacyjnym po 100 epokach uczenia.

| Miara | Wartość | Interpretacja |
| :--- | :--- | :--- |
| **Precision** | 0.857 | 85.7% wykrytych obiektów faktycznie istnieje. |
| **Recall** | 0.543 | System wykrywa ok. 54% wszystkich obiektów w scenie. |
| **mAP50** | 0.627 | Średnia precyzja przy progu IoU 0.5. |
| **mAP50-95** | 0.418 | Precyzja przy rygorystycznych progach dopasowania. |

**Wniosek**: Model wykazuje wysoką precyzję (mało fałszywych alarmów), ale niższą czułość (Recall), co sugeruje trudności z wykrywaniem bardzo małych (dalekich) obiektów.
<br>

###  Dokładność Estymacji Odległości
Wyniki porównano z obiektywnymi danymi Ground Truth 3D z symulatora w rozszerzonym teście na próbie 100 klatek (kilkaset indywidualnych pomiarów obiektów).

*   **Średni błąd bezwzględny (MAE)**: **1.40 metra**.
*   **Strefa krytyczna (do 15m)**: Błąd zazwyczaj poniżej **0.5 metra**. Bardzo wysoka precyzja w obszarze kluczowym dla bezpieczeństwa.
*   **Strefa daleka (>15m)**: Błąd rośnie do 1-2m (wynika z ograniczeń rozdzielczości obrazu - 1 piksel błędu bounding boxa mocno wpływa na wynik).
*   **Wartości odstające (Outliery)**: Pojedyncze błędy >5m wynikają głównie z zagęszczenia obiektów i problemów z asocjacją w scenach o dużym tłoku.


### Analiza Błędów i Ograniczeń
1.  **Konsolidacja Obiektów (Motorocyclist)**: W procesie przygotowania zbioru danych zastosowano celową fuzję ramek ograniczających dla motocykla i motocyklisty. Dzięki temu system traktuje ich jako jeden spójny obiekt NURD. Eliminuje to redundancję w module śledzenia (jedno ID na jeden pojazd) i stabilizuje estymację dystansu, która opiera się na sumarycznej wysokości sylwetki.
2.  **Błąd "Standardowego Wzrostu"**: Największe odchylenia w module dystansu wynikają z różnic między założonym wzrostem statystycznym (np. 1.70m) a faktycznym modelem 3D w CARLA. Odchylenia te są jednak akceptowalne.
2.  **Ograniczenia Kamery Monokularnej**: Zastosowany wzór $Z = (H_{real} \cdot f) / h_{px}$ jest bardzo wrażliwy na precyzję wysokości bounding boxa, stąd spadek dokładności na dużych dystansach.
3.  **Recall (Czułość)**: Recall na poziomie 0.54 wskazuje, że dalekie lub przysłonięte obiekty mogą być pomijane. Wymaga to użycia modeli o wyższej rozdzielczości wejściowej w przyszłości.

### Wnioski Końcowe
*   System wykazuje **wysoką stabilność numeryczną**. Błąd MAE na poziomie 1.40m przy zastosowaniu prostej kamery (bez LiDARa) to wynik bardzo dobry, zwłaszcza że w strefie krytycznej (blisko pojazdu) dokładność wynosi kilkanaście centymetrów.
*   System jest **wystarczający dla decyzji typu "Zwolnij/Hamuj"** (ADAS), opierając się na trendzie zmiany dystansu (malejący TTC).
*   Integracja z CARLA poprzez interfejs `NURDApp` (z uwzględnieniem metrycznej prędkości zbliżania) czyni system gotowym do testów w zamkniętej pętli sterowania (Closed-Loop).

