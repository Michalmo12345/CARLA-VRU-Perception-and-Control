# Dokumentacja Algorytmów i Fizyki Systemu NURD

Niniejszy dokument opisuje kluczowe algorytmy, modele matematyczne oraz wzory fizyczne zastosowane w potoku przetwarzania systemu wykrywania Niechronionych Uczestników Ruchu Drogowego (NURD).

---

## 1. Moduł Śledzenia (Tracking Module)

Śledzenie obiektów opiera się na dwóch głównych filarach: asocjacji danych (Data Association) oraz filtracji trajektorii za pomocą Filtra Kalmana.

### 1.1. Filtr Kalmana (Model Stałej Prędkości)
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

## 2. Moduł Estymacji Odległości (Distance Estimation)

Wykorzystuje model kamery otworkowej (**Pinhole Camera Model**) do przejścia z 2D do 3D.

*   **Wzór główny**:
    $$ Z = \frac{H_{real} \cdot f}{h_{pixel}} $$

    Gdzie:
    *   $Z$ - Odległość wzdłuż osi optycznej [metry].
    *   $H_{real}$ - Fizyczna wysokość obiektu [metry] (Priors: Pieszy 1.7m, Rowerzysta 1.65m, Motorower 1.5m).
    *   $f$ - Ogniskowa kamery [piksele].
    *   $h_{pixel}$ - Wysokość ramki na obrazie [piksele].

---

## 3. Moduł Oceny Ryzyka i Kinematyki Przestrzennej

### 3.1. Prędkość Zbliżania (Radial Approach Velocity)
Wyliczana jako pochodna dystansu fizycznego $Z$ w czasie:
$$ v_{app} = \frac{Z_{t - \Delta t} - Z_{t}}{\Delta t} $$

### 3.2. Czas do Kolizji (Time To Collision - TTC)
Określa czas do zderzenia przy założeniu stałej prędkości zbliżania:
$$ TTC = \frac{Z_t}{v_{app}} $$
*(Dla $v_{app} \le 0$ przyjmuje się $TTC = \infty$)*.

### 3.3. Logika Decyzyjna
*   **CRITICAL**: $TTC < 1.5\text{s}$ lub $Z < 5\text{m}$ $\rightarrow$ Hamowanie awaryjne ($V_{target} = 0$).
*   **HIGH**: $TTC < 4.0\text{s}$ $\rightarrow$ Mocne zwolnienie ($V_{target} = 30\% \cdot V_{base}$).
*   **MEDIUM**: $Z < 20\text{m}$ $\rightarrow$ Lekkie zwolnienie ($V_{target} = 70\% \cdot V_{base}$).
*   **LOW**: Pozostałe $\rightarrow$ Utrzymanie prędkości ($V_{target} = V_{base}$).
