import numpy as np
from typing import Dict, List, Tuple, Optional
from scipy.optimize import linear_sum_assignment

class KalmanFilter:
    """
    KF dla modelu Constant Velocity [x, y, vx, vy].
    Obserwujemy tylko pozycję [x, y].
    """
    def __init__(self, dt: float, q_noise: float = 0.1, r_noise: float = 0.03):
        self.dt = dt
        
        # State transition matrix
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        self.Q = np.eye(4, dtype=np.float32) * q_noise
        self.R = np.eye(2, dtype=np.float32) * r_noise
        self.P = np.eye(4, dtype=np.float32) * 1.0
        self.x = np.zeros((4, 1), dtype=np.float32)

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z: np.ndarray):
        z = z.reshape(2, 1)
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(4) - np.dot(K, self.H), self.P)

class TrackedObject:
    """
    Pojedynczy obiekt śledzony z estymacją stanu.
    """
    def __init__(self, track_id: int, class_id: int, pos: np.ndarray, dim: Tuple[float, float], dt: float):
        self.track_id = track_id
        self.class_id = class_id
        self.kf = KalmanFilter(dt)
        self.kf.x[0:2] = pos.reshape(2, 1)
        
        self.hits = 1
        self.time_since_update = 0
        
        self.cx, self.cy = pos
        self.w, self.h = dim
        self.vx, self.vy = 0.0, 0.0
        self.head = 0.0

    def update(self, pos: np.ndarray, dim: Tuple[float, float]):
        self.time_since_update = 0
        self.hits += 1
        self.w, self.h = dim
        
        self.kf.predict()
        self.kf.update(pos)
        
        self.cx, self.cy = self.kf.x[0, 0], self.kf.x[1, 0]
        self.vx, self.vy = self.kf.x[2, 0], self.kf.x[3, 0]
        
        if np.hypot(self.vx, self.vy) > 0.1:
            self.head = np.arctan2(self.vy, self.vx)

    def predict(self):
        self.kf.predict()
        self.cx, self.cy = self.kf.x[0, 0], self.kf.x[1, 0]
        self.vx, self.vy = self.kf.x[2, 0], self.kf.x[3, 0]
        self.time_since_update += 1

class TrackingModule:
    """
    Centroid tracking z minimalizacją kosztów (Linear Sum Assignment).
    """
    def __init__(self, max_age: int = 10, min_hits: int = 3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.track_id_counter = 0
        self.tracks: List[TrackedObject] = []

    def update(self, detections: np.ndarray, dt: float) -> np.ndarray:
        """
        Główna pętla trackera. 
        Zwraca: [track_id, cx, cy, w, h, vx, vy, head, class_id]
        """
        for track in self.tracks:
            track.predict()
            track.kf.dt = dt

        if len(detections) > 0 and len(self.tracks) > 0:
            det_centroids = np.array([( (d[0]+d[2])/2, (d[1]+d[3])/2 ) for d in detections])
            track_centroids = np.array([(t.cx, t.cy) for t in self.tracks])
            
            from scipy.spatial import distance
            dist_matrix = distance.cdist(det_centroids, track_centroids, 'euclidean')
            
            rows, cols = linear_sum_assignment(dist_matrix)
            
            assigned_dets = set()
            for r, c in zip(rows, cols):
                if dist_matrix[r, c] < 120.0: # Próg dystansu asocjacji
                    det = detections[r]
                    self.tracks[c].update(det_centroids[r], (det[2]-det[0], det[3]-det[1]))
                    self.tracks[c].class_id = int(det[5])
                    assigned_dets.add(r)
        else:
            assigned_dets = set()

        # Init nowych obiektów
        for i, det in enumerate(detections):
            if i not in assigned_dets:
                cx, cy = (det[0]+det[2])/2, (det[1]+det[3])/2
                w, h = det[2]-det[0], det[3]-det[1]
                new_track = TrackedObject(self.track_id_counter, int(det[5]), np.array([cx, cy]), (w, h), dt)
                self.tracks.append(new_track)
                self.track_id_counter += 1

        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]

        output = []
        for t in self.tracks:
            if t.hits >= self.min_hits or t.time_since_update == 0:
                output.append([t.track_id, t.cx, t.cy, t.w, t.h, t.vx, t.vy, t.head, t.class_id])

        return np.array(output) if output else np.empty((0, 9))
