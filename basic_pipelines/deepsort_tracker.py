import numpy as np
from scipy.optimize import linear_sum_assignment
from collections import deque
from filterpy.kalman import KalmanFilter

class Track:
    def __init__(self, track_id, bbox, max_age=30):
        self.track_id = track_id
        self.bbox = bbox
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.eye(7)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0]
        ])
        self.kf.R[2:, 2:] *= 10.
        self.kf.P[4:, 4:] *= 1000.
        self.kf.P *= 10.
        self.kf.Q[-1, -1] *= 0.01
        self.kf.Q[4:, 4:] *= 0.01
        self.age = 0
        self.time_since_update = 0
        self.hits = 0
        self.hit_streak = 0
        self.history = deque(maxlen=max_age)
        
    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y
  
    def update(self, bbox):
        self.kf.update(np.array(bbox))
        self.bbox = self.kf.x[:4]
        self.age += 1
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        self.history.append(bbox)

    def predict(self):
        self.kf.predict()
        self.bbox = self.kf.x[:4]
        self.age += 1
        self.time_since_update += 1
        if self.time_since_update > 0:
            self.hit_streak = 0

class DeepSORTTracker:
    def __init__(self, max_age=30, min_hits=3, iou_threshold=0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks = []
        self.next_id = 1

    def iou(self, bbox1, bbox2):
        x1, y1, x2, y2 = bbox1
        x1_p, y1_p, x2_p, y2_p = bbox2

        xi1 = max(x1, x1_p)
        yi1 = max(y1, y1_p)
        xi2 = min(x2, x2_p)
        yi2 = min(y2, y2_p)
        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)

        bbox1_area = (x2 - x1) * (y2 - y1)
        bbox2_area = (x2_p - x1_p) * (y2_p - y1_p)
        union_area = bbox1_area + bbox2_area - inter_area

        return inter_area / union_area

    def update(self, detections):
        if len(self.tracks) == 0:
            for det in detections:
                self.tracks.append(Track(self.next_id, det))
                self.next_id += 1
            return

        iou_matrix = np.zeros((len(self.tracks), len(detections)), dtype=np.float32)
        for t, track in enumerate(self.tracks):
            for d, det in enumerate(detections):
                iou_matrix[t, d] = self.iou(track.bbox, det)

        row_ind, col_ind = linear_sum_assignment(-iou_matrix)

        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections)))

        for t, d in zip(row_ind, col_ind):
            if iou_matrix[t, d] < self.iou_threshold:
                unmatched_tracks.add(t)
                unmatched_detections.add(d)
            else:
                unmatched_tracks.discard(t)
                unmatched_detections.discard(d)
                self.tracks[t].update(detections[d])

        for t in unmatched_tracks:
            self.tracks[t].predict()

        for d in unmatched_detections:
            self.tracks.append(Track(self.next_id, detections[d]))
            self.next_id += 1

        self.tracks = [t for t in self.tracks if t.time_since_update < self.max_age]

    def get_tracks(self):
        return [t for t in self.tracks if t.hits >= self.min_hits or t.age < self.min_hits]