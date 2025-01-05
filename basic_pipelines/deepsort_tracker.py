import numpy as np
from scipy.optimize import linear_sum_assignment
from collections import deque

# === KALMAN FILTER ===
class KalmanFilter2D:
    """
    Filtre de Kalman simplifié en 2D :
    État : [x, y, vx, vy]
    Mesure : [x, y]
    """

    def __init__(self, dt=1/25):
        """
        dt : intervalle de temps (approx. 1/fps).
        """
        self.dt = dt

        # État : [x, y, vx, vy] (4x1)
        self.x = np.zeros((4, 1), dtype=np.float32)

        # Matrice de transition (F)
        self.F = np.array([
            [1, 0, self.dt,      0],
            [0, 1,      0, self.dt],
            [0, 0,      1,      0],
            [0, 0,      0,      1]
        ], dtype=np.float32)

        # Matrice d’observation (H)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        # Incertitude du modèle (Q) et bruit de mesure (R)
        self.Q = np.eye(4, dtype=np.float32) * 2
        self.R = np.eye(2, dtype=np.float32) * 50

        # Matrices de covariance (P)
        self.P = np.eye(4, dtype=np.float32)

    def predict(self):
        """
        Étape de prédiction : x(k) = F * x(k-1)
        """
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        """
        Étape de mise à jour (z = [x_mesure, y_mesure]).
        """
        z = np.array(z, dtype=np.float32).reshape((2, 1))
        # Innovation = z - Hx
        y = z - np.dot(self.H, self.x)
        # S = H P H^T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        # K = P H^T S^(-1)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # x_new = x + K y
        self.x = self.x + np.dot(K, y)
        # P_new = (I - K H) P
        I = np.eye(self.P.shape[0], dtype=np.float32)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

    def get_estimated_position(self):
        """
        Retourne (x, y) du vecteur d'état courant.
        """
        return float(self.x[0]), float(self.x[1])

    def set_initial_position(self, x, y):
        """
        Initialise la position directement (optionnel).
        """
        self.x[0] = x
        self.x[1] = y

class Track:
    def __init__(self, track_id, detection, dt=1/25):
        """
        Initialise une nouvelle piste avec une détection initiale.
        
        :param track_id: Identifiant unique de la piste.
        :param detection: Bounding box initiale [x1, y1, x2, y2].
        :param dt: Intervalle de temps entre les mises à jour.
        """
        self.track_id = track_id
        self.kalman_filter = KalmanFilter2D(dt=dt)
        
        # Calculer le centre de la bounding box initiale
        x1, y1, x2, y2 = detection
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        self.kalman_filter.set_initial_position(center_x, center_y)
        
        self.age = 0
        self.time_since_update = 0
        self.hits = 1
        self.hit_streak = 1
        self.history = deque(maxlen=50)  # Historique des positions estimées
        
        self.bbox = detection  # Dernière bounding box observée

    def predict(self):
        """
        Prédire la prochaine position de la piste en utilisant le filtre de Kalman.
        """
        self.kalman_filter.predict()
        self.age += 1
        self.time_since_update += 1
        # Stocker l'estimation pour l'historique si nécessaire
        estimated_position = self.kalman_filter.get_estimated_position()
        self.history.append(estimated_position)
        return estimated_position

    def update(self, detection):
        """
        Mettre à jour la piste avec une nouvelle détection.
        
        :param detection: Nouvelle bounding box détectée [x1, y1, x2, y2].
        """
        x1, y1, x2, y2 = detection
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        self.kalman_filter.update([center_x, center_y])
        self.bbox = detection
        self.hits += 1
        self.hit_streak += 1
        self.time_since_update = 0

    @property
    def center(self):
        """
        Propriété pour accéder au centre estimé de la piste.
        """
        return self.kalman_filter.get_estimated_position()

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
        Calculer l'Intersection over Union (IoU) entre deux bounding boxes.
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
        # Si aucune piste n'existe, initialiser toutes les détections comme nouvelles pistes
        if len(self.tracks) == 0:
            for det in detections:
                self.tracks.append(Track(self.next_id, det, dt=self.dt))
                self.next_id += 1
            return

        # Prédire la position de toutes les pistes existantes
        predicted_positions = []
        for track in self.tracks:
            predicted_pos = track.predict()
            predicted_positions.append(predicted_pos)

        # Calculer la matrice de coût basée sur la distance euclidienne entre positions prédites et détections
        cost_matrix = np.zeros((len(self.tracks), len(detections)), dtype=np.float32)
        for t, track in enumerate(self.tracks):
            for d, det in enumerate(detections):
                x1, y1, x2, y2 = det
                det_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                track_center = np.array(predicted_positions[t])
                distance = np.linalg.norm(det_center - track_center)
                cost_matrix[t, d] = distance

        # Appliquer l'algorithme de Hungarian pour minimiser le coût total
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Définir un seuil maximal de distance pour considérer une association valide
        max_distance = 50  # Vous pouvez ajuster ce paramètre

        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections)))

        for t, d in zip(row_ind, col_ind):
            if cost_matrix[t, d] > max_distance:
                unmatched_tracks.add(t)
                unmatched_detections.add(d)
            else:
                # Associer la détection à la piste
                self.tracks[t].update(detections[d])
                unmatched_tracks.discard(t)
                unmatched_detections.discard(d)

        # Gérer les pistes non associées
        for t in unmatched_tracks:
            self.tracks[t].time_since_update += 1
            self.tracks[t].hit_streak = 0

        # Créer de nouvelles pistes pour les détections non associées
        for d in unmatched_detections:
            self.tracks.append(Track(self.next_id, detections[d], dt=self.dt))
            self.next_id += 1

        # Supprimer les pistes qui ont dépassé le nombre maximal de frames sans mise à jour
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]

    def get_tracks(self):
        """
        Retourner les pistes actives qui ont suffisamment de hits ou qui sont récentes.

        :return: Liste de pistes actives.
        """
        return [t for t in self.tracks if t.hits >= self.min_hits or t.time_since_update < self.min_hits]