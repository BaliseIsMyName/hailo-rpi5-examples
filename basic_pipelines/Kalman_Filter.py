import numpy as np
from scipy.optimize import linear_sum_assignment

class EKF:
    def __init__(self, state_dim=7, measure_dim=4, dt=1/25):
        """
        Initialise l'EKF.
        :param state_dim: Dimension de l'état [x, y, w, h, vx, vy, z]
        :param measure_dim: Dimension des mesures [x, y, w, h]
        :param dt: Intervalle de temps entre les mises à jour
        """
        self.state_dim = state_dim
        self.measure_dim = measure_dim
        self.dt = dt

        # Vecteur d'état
        self.x = np.zeros((state_dim, 1))

        # Matrices du modèle
        self.F = np.eye(state_dim)  # Transition
        self.H = np.zeros((measure_dim, state_dim))  # Observation
        self.Q = np.eye(state_dim) * 0.01  # Bruit de processus
        self.R = np.eye(measure_dim) * 1.0  # Bruit d'observation
        self.P = np.eye(state_dim) * 10.0  # Covariance initiale

        # Configuration des matrices
        self.configure_matrices()

    def configure_matrices(self):
        """
        Configure les matrices pour le modèle EKF.
        """
        dt = self.dt
        # Modèle de mouvement constant pour la transition
        self.F[:2, 2:4] = np.eye(2) * dt  # vx, vy affectent x, y
        # Modèle de mesure : [x, y, w, h] observés
        self.H[:4, :4] = np.eye(4)

    def predict(self, u=None):
        """
        Étape de prédiction.
        :param u: Commande externe (facultatif)
        """
        if u is None:
            u = np.zeros((self.state_dim, 1))
        self.x = self.F @ self.x + u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """
        Étape de mise à jour.
        :param z: Observation [x, y, w, h]
        """
        z = np.reshape(z, (self.measure_dim, 1))
        y = z - self.H @ self.x  # Innovation
        S = self.H @ self.P @ self.H.T + self.R  # Covariance de l'innovation
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Gain de Kalman
        self.x = self.x + K @ y  # Mise à jour de l'état
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P  # Mise à jour de la covariance

    def get_state(self):
        """
        Retourne l'état actuel sous forme de vecteur.
        """
        return self.x.flatten()


class Track:
    def __init__(self, track_id, bbox, dt=1/25):
        """
        Initialise une piste pour un objet.
        :param track_id: Identifiant unique de la piste
        :param bbox: Bounding box initiale [x, y, w, h]
        :param dt: Intervalle de temps pour l'EKF
        """
        self.id = int(track_id)
        self.ekf = EKF(state_dim=7, measure_dim=4, dt=dt)
        self.ekf.update(bbox)  # Initialiser avec la bounding box
        self.time_since_update = 0
        self.hits = 1
        self.hit_streak = 1
        self.age = 0

    def predict(self):
        """
        Prédit la prochaine position avec l'EKF.
        """
        self.ekf.predict()
        self.age += 1
        self.time_since_update += 1
        return self.ekf.get_state()

    def update(self, bbox):
        """
        Met à jour la piste avec une nouvelle bounding box.
        :param bbox: Bounding box [x, y, w, h]
        """
        self.ekf.update(bbox)
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1

    def get_state(self):
        """
        Retourne l'état actuel de la piste.
        """
        return self.ekf.get_state()

    def get_center(self, width, height):
      """
      Retourne les coordonnées du centre de la bounding box normalisées (0 à 1).
      """
      state = self.get_state()  # [x, y, w, h, vx, vy, z]
      center_x = state[0] / width
      center_y = state[1] / height
      return center_x, center_y


class SORT:
      def __init__(self, max_age=30, min_hits=3, iou_threshold=0.3):
            """
            Initialise le tracker SORT.
            :param max_age: Nombre maximal de frames sans mise à jour avant suppression
            :param min_hits: Nombre minimal de hits avant affichage
            :param iou_threshold: Seuil IoU pour l'association
            """
            self.max_age = max_age
            self.min_hits = min_hits
            self.iou_threshold = iou_threshold
            self.tracks = []
            self.next_id = 1

      def iou(self, bbox1, bbox2):
            """
            Calculer l'Intersection over Union (IoU) entre deux bounding boxes.
            :param bbox1: [xmin, ymin, xmax, ymax]
            :param bbox2: [xmin, ymin, xmax, ymax]
            :return: IoU (float)
            """
            x1, y1, x2, y2 = bbox1
            x1_p, y1_p, x2_p, y2_p = bbox2

            xi1 = max(x1, x1_p)
            yi1 = max(y1, y1_p)
            xi2 = min(x2, x2_p)
            yi2 = min(y2, y2_p)

            inter_width = max(0, xi2 - xi1)
            inter_height = max(0, yi2 - yi1)
            inter_area = inter_width * inter_height

            bbox1_area = (x2 - x1) * (y2 - y1)
            bbox2_area = (x2_p - x1_p) * (y2_p - y1_p)
            union_area = bbox1_area + bbox2_area - inter_area

            return inter_area / union_area if union_area > 0 else 0.0

      def update(self, detections):
            """
            Mettre à jour les pistes avec les nouvelles détections.
            :param detections: Liste des bounding boxes détectées [xmin, ymin, xmax, ymax]
            """
            if len(self.tracks) == 0:
                  print(f"Aucune piste existante. Création de pistes pour {len(detections)} détections.")
                  for det in detections:
                        print(f"Création d'une nouvelle piste pour : {det}")
                        self.tracks.append(Track(self.next_id, det))
                        self.next_id += 1
                  return

            # Prédictions et matrice de coût
            predicted_states = [track.predict()[:4] for track in self.tracks]
            cost_matrix = np.zeros((len(self.tracks), len(detections)), dtype=np.float32)
            for t, track in enumerate(self.tracks):
                  print(f"Piste {t}, bbox : {track.get_state()[:4]}")
                  for d, det in enumerate(detections):
                        cost_matrix[t, d] = 1 - self.iou(predicted_states[t], det)
            
            print(f"Matrix de coût : \n{cost_matrix}")

            # Hungarian Algorithm pour l'association
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            unmatched_tracks = set(range(len(self.tracks)))
            unmatched_detections = set(range(len(detections)))

            for t, d in zip(row_ind, col_ind):
                  if cost_matrix[t, d] > 1 - self.iou_threshold:
                        unmatched_tracks.add(t)
                        unmatched_detections.add(d)
                  else:
                        print(f"Association : Piste {t} -> Détection {d}")
                        self.tracks[t].update(detections[d])
                        unmatched_tracks.discard(t)
                        unmatched_detections.discard(d)

            print(f"Tracks non associées : {unmatched_tracks}")
            print(f"Detections non associées : {unmatched_detections}")

            # Mise à jour des pistes non associées
            for t in unmatched_tracks:
                  self.tracks[t].time_since_update += 1

            # Ajouter de nouvelles pistes pour les détections non associées
            for d in unmatched_detections:
                  print(f"Création d'une nouvelle piste pour la détection : {detections[d]}")
                  self.tracks.append(Track(self.next_id, detections[d]))
                  self.next_id += 1

            # Supprimer les pistes obsolètes
            print(f"Pistes avant suppression : {len(self.tracks)}")
            self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
            print(f"Pistes après suppression : {len(self.tracks)}")

      def get_tracks(self):
            """
            Retourner les pistes actives.
            """
            return [t for t in self.tracks if t.hits >= self.min_hits]
