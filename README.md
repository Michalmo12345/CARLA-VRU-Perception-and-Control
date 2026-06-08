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

---

# CARLA Simulation (NURD VRU Perception & Control)

Closed-loop integration with **CARLA 0.9.16**: the trained YOLO model perceives
VRUs from a car-mounted camera, the NURD pipeline estimates distance / time-to-collision
and assigns a risk level, and the vehicle brakes accordingly while driving a fixed route.

Pipeline: **YOLO detection → tracking (Kalman) → distance (pinhole) → risk → braking control**.

## Prerequisites

- A running CARLA 0.9.16 server (it can run on a Windows host while the client runs in WSL2).
- The CARLA Python API installed in the environment:
  ```bash
  pip install carla==0.9.16
  ```
- For the live OpenCV window (without `--headless`) under WSL, the Qt `xcb` plugin needs
  system libraries:
  ```bash
  sudo apt update && sudo apt install -y libsm6 libice6 libxcb-cursor0 libxcb-xinerama0
  ```

## Quick check: server connection

```bash
python test_carla_connection.py --host 127.0.0.1
```

## Running the simulation

Entry point: `nurd_carla_simulation.py` (thin wrapper over `src/simulation/runner.py`).

```bash
# CARLA on the same host
python nurd_carla_simulation.py

# CARLA on a Windows host, client in WSL2 (use the host IP, or 127.0.0.1 with mirrored networking)
python nurd_carla_simulation.py --host 127.0.0.1

# No OpenCV window (recommended in WSL if Qt libs are missing)
python nurd_carla_simulation.py --host 127.0.0.1 --headless
```

### Common scenarios

```bash
# Rain / night to test perception robustness
python nurd_carla_simulation.py --host 127.0.0.1 --weather rain
python nurd_carla_simulation.py --host 127.0.0.1 --weather night

# Put a road sign on the lane near the end (object outside YOLO classes -> car won't react)
python nurd_carla_simulation.py --host 127.0.0.1 --end-sign

# Detection only, no braking
python nurd_carla_simulation.py --host 127.0.0.1 --no-brake

# Free pedestrians walking on the sidewalks (off by default)
python nurd_carla_simulation.py --host 127.0.0.1 --free-pedestrians 6
```

### Command-line flags

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | `localhost` | CARLA server address (Windows host IP from WSL2) |
| `--port` | `2000` | CARLA server port |
| `--model` | `runs/detect/TWM/run/weights/best.pt` | YOLO weights (falls back to `yolo11s.pt`) |
| `--map` | `Town02_Opt` | CARLA map, or `auto` to scan for a straight route |
| `--route-length` | `90` | Straight route length [m] |
| `--cruise-speed` | `20` | Target speed with no hazard [km/h] |
| `--cyclist-speed` | `10` | Cyclist/motorcycle crossing speed [km/h] |
| `--pedestrian-speed` | `3.0` | Pedestrian walking speed [m/s] |
| `--approach-seconds` | `4` | Seconds a VRU walks from the curb onto the lane |
| `--pause-on-lane` | `2` | Seconds a VRU stops on the lane before leaving |
| `--weather` | `default` | `clear`/`cloudy`/`wet`/`rain`/`fog`/`night`/`rainnight` |
| `--end-sign` | off | Place a road sign on the lane at ~95% of the route |
| `--fuse-closing-speed` | off | Fuse image-radial closing speed with ego speed (default: ego only) |
| `--free-pedestrians` | `0` | Pedestrians strolling on sidewalks |
| `--camera-view` | `hood` | `hood` (bumper) or `driver` (cabin) |
| `--sim-delta` | `0.1` | Fixed simulation timestep [s] |
| `--no-brake` | off | Detection/visualization only, no vehicle control |
| `--no-sync-spectator` | off | Don't follow the car camera in the CARLA window |
| `--headless` | off | No OpenCV window (CARLA window still shows the camera) |
| `--frames` | `0` | Stop after N frames (0 = unlimited) |

### Scenario

Five sequential crossing scenes are spawned along the route (each triggers after the
previous finishes): **pedestrian → cyclist → pedestrian → pedestrian → motorcycle**.
Each VRU walks/rides from the curb onto the lane, pauses, then crosses.

## Simulation structure

- `nurd_carla_simulation.py` — thin entry point (CARLA import guard + `run()`).
- `src/simulation/` — simulation package:
  - `config.py` — constants, camera presets, scenario sequence.
  - `cli.py` — argument parsing.
  - `geometry.py` — pure geometry helpers (yaw, lerp, distances).
  - `route_planner.py` — map loading, straight-route finding, sidewalks, vehicle spawn.
  - `crossing.py` — crossing geometry and snapping to the driving surface.
  - `vru_movement.py` — VRU movement strategies (animated walkers, teleport for vehicles).
  - `vru_spawner.py` — VRU spawning + grounding to the surface.
  - `scenario.py` — scene sequencing.
  - `weather.py` — weather presets.
  - `props.py` — static obstacles (road sign).
  - `hud.py` — status overlay.
  - `runner.py` — `CarlaSimulation` orchestration loop.
- `src/nurd_system/` — perception/decision modules: `detection`, `tracking`, `distance`, `risk`, `control`.
- `nurd_app.py` — NURD pipeline wiring (also a standalone webcam test via `main()`).

## Changelog (since last commit)

- **Modularization**: split the ~1300-line `nurd_carla_simulation.py` monolith into a
  cohesive `src/simulation` package with a thin entry point (SRP / separation of concerns).
- **Risk model overhaul** (`src/nurd_system/risk.py`):
  - pinhole-based collision *corridor* in meters — the whole frame is analyzed
    (sidewalks included), but braking only triggers when the VRU path actually crosses
    the car's path;
  - EMA-smoothed distance and closing speed (removes the noise that caused constant CRITICAL);
  - TTC and required deceleration based on the exact CARLA ego speed;
  - smooth, distance-based target-speed profile.
- **Smoother braking** (`src/nurd_system/control.py`): proportional braking with a
  target deadband and low-pass smoothing, coasting under risk (no throttle/brake fight),
  full brake reserved for CRITICAL — removes the previous judder.
- **VRU grounding fix**: actor Z is aligned to the driving surface from its bounding box
  instead of a fixed offset (no more sinking through the road).
- **Natural VRU motion**: pedestrians are driven by `carla.WalkerControl` (real walk
  animation, no T-pose), including those strolling on sidewalks; vehicles (cyclist/motorcycle)
  use transform-based crossing.
- **Scenario**: 5 sequential scenes, last one is a **motorcycle**.
- **New flags**: `--weather`, `--end-sign`, `--fuse-closing-speed`.
- **Cleanup**: camera params injected into `NURDApp` via the constructor; dead code removed;
  added `test_carla_connection.py`.
