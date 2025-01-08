#deepsort_tracker.py    

import numpy as np
from scipy.optimize import linear_sum_assignment
from collections import deque

# === KALMAN FILTER ===
class KalmanFilterBBOX:
    """
    Filtre de Kalman pour suivre une boîte englobante complète : [x, y, vx, vy, width, height].
    """

    def __init__(self, dt=1/25):
        self.dt = dt

        # État : [x, y, vx, vy, width, height] (6x1)
        self.x = np.zeros((6, 1), dtype=np.float32)

        # Matrice de transition (F)
        self.F = np.array([
            [1, 0, self.dt, 0, 0, 0],
            [0, 1, 0, self.dt, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)

        # Matrice d'observation (H) : nous observons [x, y, width, height]
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)

        # Bruit du modèle et de la mesure
        self.Q = np.eye(6, dtype=np.float32) * 0.3
        self.R = np.eye(4, dtype=np.float32) * 3

        # Matrice de covariance (P)
        self.P = np.eye(6, dtype=np.float32)

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        z = np.array(z, dtype=np.float32).reshape((4, 1))  # z = [x, y, width, height]
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0], dtype=np.float32)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

    def get_estimated_bbox(self):
        """
        Retourne la boîte englobante estimée (x, y, width, height).
        """
        x, y, w, h = self.x[0, 0], self.x[1, 0], self.x[4, 0], self.x[5, 0]
        return x, y, w, h


    def set_initial_position(self, x, y):
        """
        Initialise la position directement (optionnel).
        """
        self.x[0] = x
        self.x[1] = y

class Track:
    def __init__(self, track_id, detection, dt=1/25):
        """
        Initialise une nouvelle piste avec une boîte englobante complète.
        
        :param track_id: Identifiant unique de la piste.
        :param detection: Bounding box initiale [x1, y1, x2, y2].
        :param dt: Intervalle de temps entre les mises à jour.
        """
        self.track_id = track_id
        self.kalman_filter = KalmanFilterBBOX(dt=dt)
        
        # Initialiser l'état avec la détection (centre + dimensions)
        x1, y1, x2, y2 = detection
        width = x2 - x1
        height = y2 - y1
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
                
        #initialisation de l'état du filtre
        self.kalman_filter.x[0, 0] = center_x  # Position X
        self.kalman_filter.x[1, 0] = center_y  # Position Y
        self.kalman_filter.x[2, 0] = 0.0        # Vitesse X initialisée à 0
        self.kalman_filter.x[3, 0] = 0.0        # Vitesse Y initialisée à 0
        self.kalman_filter.x[4, 0] = width      # Largeur
        self.kalman_filter.x[5, 0] = height     # Hauteur

        self.age = 0
        self.time_since_update = 0
        self.hits = 1
        self.hit_streak = 1
        self.history = deque(maxlen=50)
        self.bbox = detection


    def predict(self):
        """
        Prédire la prochaine position (x, y, width, height).
        Arrête les prédictions si le track est inactif trop longtemps.
        """
        if self.time_since_update > 5:  # Par exemple, après 3 frames inactives
            return None  # Ne plus prédire
        self.kalman_filter.predict()
        self.age += 1
        self.history.append(self.kalman_filter.get_estimated_bbox())
        return self.kalman_filter.get_estimated_bbox()
    
    def update(self, detection):
        """
        Mettre à jour avec une nouvelle détection.
        
        :param detection: Bounding box détectée [x1, y1, x2, y2].
        """
        x1, y1, x2, y2 = detection
        width = x2 - x1
        height = y2 - y1
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        self.kalman_filter.update([center_x, center_y, width, height])
        self.bbox = detection
        self.hits += 1
        self.hit_streak += 1
        self.time_since_update = 0

    @property
    def center(self):
        """
        Propriété pour accéder au centre estimé de la piste.
        Retourne uniquement les coordonnées x et y.
        """
        x, y, _, _ = self.kalman_filter.get_estimated_bbox()
        return x, y

class DeepSORTTracker:
    def __init__(self, max_age=30, min_hits=3, iou_threshold=0.3, dt=1/25):
        """
        Initialise le tracker DeepSORT.

        :param max_age: Nombre maximal de frames sans mise à jour avant suppression de la piste.
        :param min_hits: Nombre minimal de mises à jour réussies avant d'afficher la piste.
        :param iou_threshold: Seuil IoU pour l'association des détections.
        :param dt: Intervalle de temps entre les mises à jour pour le filtre de Kalman.
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks = []
        self.next_id = 1
        self.dt = dt

    def iou(self, bbox1, bbox2):
        """
        Calculer l'IoU entre deux boîtes englobantes.
        """
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

        return inter_area / union_area if union_area > 0 else 0

    def update(self, detections):
        """
        Mettre à jour les pistes avec les nouvelles détections.

        :param detections: Liste de bounding boxes détectées [x1, y1, x2, y2].
        """
        alpha = 0.5  # Donne un poids modéré à la distance

        if len(self.tracks) == 0:
            for det in detections:
                self.tracks.append(Track(self.next_id, det, dt=self.dt))
                self.next_id += 1
            return

        # Prédire la position de toutes les pistes existantes
        predicted_bboxes = [track.predict() for track in self.tracks]

        # Calculer la matrice de coût basée sur l'IoU
        cost_matrix = np.zeros((len(self.tracks), len(detections)), dtype=np.float32)
        for t, track in enumerate(self.tracks):
            for d, det in enumerate(detections):
                iou_cost = 1 - self.iou(track.bbox, det)  # Coût basé sur IoU
                track_center = track.center
                det_center = [
                    0.5 * (det[0] + det[2]),  # x_center
                    0.5 * (det[1] + det[3])   # y_center
                ]
                distance_cost = self.distance_metric(track_center, det_center)
                cost_matrix[t, d] = iou_cost + alpha * distance_cost

        # Appliquer l'algorithme de Hungarian pour minimiser le coût
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections)))

        for t, d in zip(row_ind, col_ind):
            if cost_matrix[t, d] > (1 - self.iou_threshold):
                unmatched_tracks.add(t)
                unmatched_detections.add(d)
            else:
                self.tracks[t].update(detections[d])
                unmatched_tracks.discard(t)
                unmatched_detections.discard(d)

        for t in unmatched_tracks:
            self.tracks[t].time_since_update += 1
            self.tracks[t].hit_streak = 0

        for d in unmatched_detections:
            self.tracks.append(Track(self.next_id, detections[d], dt=self.dt))
            self.next_id += 1

        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
        # Affichez les informations sur les tracks restants
        # for t in self.tracks:
        #     print(f"Track ID={t.track_id}, time_since_update={t.time_since_update}, age={t.age}")
        
        # print_tracks(self.tracks)

    def get_tracks(self):
        """
        Retourner uniquement les pistes actives mises à jour récemment.
        """
        return [t for t in self.tracks if t.hits >= self.min_hits or t.time_since_update == 0]
    
    @staticmethod
    def distance_metric(center1, center2):
        """
        Calculer la distance euclidienne entre deux centres.
        """
        return np.linalg.norm(np.array(center1) - np.array(center2))


def print_tracks(tracks):
    """
    Affiche les pistes actives et inactives.
    """
    active_tracks = []
    inactive_tracks = []

    for track in tracks:
        # Une piste est active si elle a été mise à jour récemment
        if track.time_since_update == 0:
            active_tracks.append(f"ID={track.track_id}, Center=({track.center[0]:.2f}, {track.center[1]:.2f})")
        else:
            inactive_tracks.append(f"ID={track.track_id}, Time Since Update={track.time_since_update}, Predicted Center=({track.center[0]:.2f}, {track.center[1]:.2f})")

    print("\n=== Pistes Actives ===")
    for active in active_tracks:
        print(active)

    print("\n=== Pistes Inactives ===")
    for inactive in inactive_tracks:
        print(inactive)

    #Fin de deepsort_tracker.py