import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
import hailo
import threading
import time
import subprocess
import socket
import json

# Imports PCA9685 & co
import board
import busio
from adafruit_pca9685 import PCA9685

from hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from detection_pipeline import GStreamerDetectionApp

# =============================
# ========= CONSTANTES ========
# =============================

TRACK_OBJECTS = ["person", "cat"]   # Labels à suivre
CONFIDENCE_THRESHOLD = 0.5          # Seuil de confiance min
DEAD_ZONE = 0.05                    # Zone morte (en fraction de l'image)

# ========================================
# =========== USER APP CALLBACK ==========
# ========================================
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.last_detections = []
        self.width = None
        self.height = None

        # Centres courants (bruts) de toutes les bbox
        self.bbox_centers = []

        # === KALMAN FILTER ===
        # Centre filtré (x, y) après Kalman
        self.kalman_filtered_center = (0.5, 0.5)  # Au départ, centre image

        # Filtre 2D: position + vitesse
        self.kalman_filter = KalmanFilter2D()
        
        self.current_fps = 0.0
        self.current_droprate = 0.0
        self.avg_fps = 0.0
        
        self.detection_event = threading.Event()
        self.lock = threading.Lock()

        self.dead_zone = DEAD_ZONE


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
        # [ x ]   [ 1  0  dt  0 ] [ x ]
        # [ y ] = [ 0  1   0  dt ] [ y ]
        # [vx ]   [ 0  0   1   0 ] [vx ]
        # [vy ]   [ 0  0   0   1 ] [vy ]
        self.F = np.array([
            [1, 0, self.dt,      0],
            [0, 1,      0, self.dt],
            [0, 0,      1,      0],
            [0, 0,      0,      1]
        ], dtype=np.float32)

        # Matrice d’observation (H)
        # On observe seulement x et y
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        # Incertitude du modèle (Q) et bruit de mesure (R)
        # À ajuster selon la fluidité / réactivité souhaitée
        self.Q = np.eye(4, dtype=np.float32) * 0.001
        self.R = np.eye(2, dtype=np.float32) * 0.01

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


# This is the callback function that will be called when data is available from the pipeline
def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Compter les frames
    user_data.increment()

    # Récupération width/height
    format, width, height = get_caps_from_pad(pad)
    if width is not None and height is not None:
        user_data.width = width
        user_data.height = height

    # Récupérer les détections
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    user_data.last_detections = detections
    # print(f'user_data : {user_data.current_fps} fps')
    # print(f"FPS: {user_data.current_fps:.2f}, Droprate: {user_data.current_droprate:.2f}, Avg FPS: {user_data.avg_fps:.2f}")
    
    # Mise à jour de la liste des centres
    calc_bbox_centers(user_data, CONFIDENCE_THRESHOLD)

    # === KALMAN FILTER ===
    # 1. Chercher la détection la plus "certaine" (label TRACK_OBJECTS + conf max)
    best_det = None
    best_conf = 0.0
    for d in detections:
        if d.get_label() in TRACK_OBJECTS:
            c = d.get_confidence()
            if c >= CONFIDENCE_THRESHOLD and c > best_conf:
                best_conf = c
                best_det = d

    # 2. On exécute le predict() du filtre à chaque frame
    user_data.kalman_filter.predict()

    # 3. S'il y a une détection valide => update() avec la position (x_center, y_center)
    if best_det is not None:
        bbox = best_det.get_bbox()
        center_x = 0.5 * (bbox.xmin() + bbox.xmax())
        center_y = 0.5 * (bbox.ymin() + bbox.ymax())
        user_data.kalman_filter.update([center_x, center_y])

    # 4. Récupérer la position estimée pour usage ultérieur
    kx, ky = user_data.kalman_filter.get_estimated_position()
    user_data.kalman_filtered_center = (kx, ky)

    # Optionnel : on peut ici appeler la logique de pilotage de caméra
    # camera_deplacement.update_position(kx, ky)

    return Gst.PadProbeReturn.OK


def calc_bbox_centers(user_data, confidence_threshold=CONFIDENCE_THRESHOLD):
    """
    Parcourt user_data.last_detections pour calculer et stocker DANS user_data.bbox_centers
    les centres (normalisés 0..1) de toutes les bounding boxes valides 
    (label suivi, conf >= threshold).
    """
    user_data.bbox_centers = []

    detections = user_data.last_detections
    if not detections:
        return

    width = user_data.width
    height = user_data.height
    if width is None or height is None:
        return

    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        if label not in TRACK_OBJECTS or confidence < confidence_threshold:
            continue

        bbox = detection.get_bbox()
        center_x = 0.5 * (bbox.xmin() + bbox.xmax())
        center_y = 0.5 * (bbox.ymin() + bbox.ymax())

        # Arrondi pour la liste d'affichage (pas forcément nécessaire)
        cx_norm = round(center_x, 4)
        cy_norm = round(center_y, 4)
        user_data.bbox_centers.append((cx_norm, cy_norm))


def draw_overlay(cairooverlay, cr, timestamp, duration, user_data):
    """
    Affiche la zone morte (cercle) + le(s) barycentre(s) (point rouge) si dispo.
    Ajoute aussi le point filtré du Kalman (vert) pour comparaison.
    """
    if user_data.width is None or user_data.height is None:
        return

    width = user_data.width
    height = user_data.height

    # Dessin du cercle bleu (zone morte)
    min_dimension = min(width, height)
    radius = user_data.dead_zone * min_dimension
    cr.set_source_rgb(0, 0, 1)  # Bleu
    cr.arc(width / 2, height / 2, radius, 0, 2 * 3.14159)
    cr.stroke()

    # Affichage de TOUS les barycentres bruts en rouge
    cr.set_source_rgb(1, 0, 0)  # Rouge
    for (center_x, center_y) in user_data.bbox_centers:
        bx = center_x * width
        by = center_y * height
        cr.arc(bx, by, 5, 0, 2 * 3.14159)
        cr.fill()

    # === KALMAN FILTER ===
    # Dessin du centre filtré en vert
    kx, ky = user_data.kalman_filtered_center
    bx_f = kx * width
    by_f = ky * height
    cr.set_source_rgb(0, 1, 0)  # Vert
    cr.arc(bx_f, by_f, 4, 0, 2 * 3.14159)
    cr.fill()


def move_window():
    """
    Déplace la fenêtre 'Hailo Detection App' en (440,62).
    """
    while True:
        try:
            window_ids = subprocess.check_output(
                ['xdotool', 'search', '--name', 'Hailo Detection App']
            ).decode().strip().split('\n')
            if window_ids:
                window_id = window_ids[0]
                subprocess.run(['xdotool', 'windowmove', window_id, '440', '62'])
                print(f"Fenêtre déplacée : ID {window_id} vers (440, 62)")
                break
            else:
                print("Fenêtre 'Hailo Detection App' non trouvée, tentative suivante...")
        except subprocess.CalledProcessError:
            print("Erreur lors de la recherche de la fenêtre 'Hailo Detection App', tentative suivante...")
        time.sleep(3)

# ========================================
# =========== PILOTAGE CAMERA ============
# ========================================
class PID:
    """
    Implémentation simple d'un PID.
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.5, output_limits=(-999, 999)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def update(self, measurement):
        now = time.time()
        dt = now - self._last_time
        if dt <= 0.0:
            dt = 1e-16

        error = self.setpoint - measurement
        p_out = self.kp * error
        self._integral += error * dt
        i_out = self.ki * self._integral
        derivative = (error - self._last_error) / dt
        d_out = self.kd * derivative

        output = p_out + i_out + d_out
        min_out, max_out = self.output_limits
        output = max(min_out, min(output, max_out))

        self._last_error = error
        self._last_time = now

        return output


class ServoController:
    """
    Pilotage d'un servo via PCA9685.
    """
    def __init__(
        self, 
        channel=0, 
        freq=50, 
        i2c_address=0x40, 
        servo_min_us=500, 
        servo_max_us=2500, 
        max_angle=180
    ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=i2c_address)
        self.pca.frequency = freq

        self.channel = channel
        self.servo_min_us = servo_min_us
        self.servo_max_us = servo_max_us
        self.max_angle = max_angle

        self.current_angle = max_angle / 2.0
        self.set_servo_angle(self.current_angle)

    def _us_to_duty_cycle(self, pulse_us):
        period_us = 1_000_000 // self.pca.frequency  # ex: 20_000µs @ 50Hz
        duty_cycle = int((pulse_us / period_us) * 65535)
        return max(0, min(65535, duty_cycle))

    def set_servo_angle(self, angle_deg):
        angle_clamped = max(0, min(self.max_angle, angle_deg))
        self.current_angle = angle_clamped

        span_us = self.servo_max_us - self.servo_min_us
        pulse_us = self.servo_min_us + (span_us * (angle_clamped / float(self.max_angle)))
        self.pca.channels[self.channel].duty_cycle = self._us_to_duty_cycle(pulse_us)

    def cleanup(self):
        self.pca.channels[self.channel].duty_cycle = 0
        self.pca.deinit()


class CameraDeplacement:
    """
    Gère deux servos (horizontal + vertical) avec 2 PID.
    """
    def __init__(
        self,
        p_horizontal=1.0, i_horizontal=0.0, d_horizontal=0.0,
        p_vertical=1.0, i_vertical=0.0, d_vertical=0.0,
        dead_zone=0.05,
        vertical_min_angle=45,
        vertical_max_angle=135,
        horizontal_min_angle=0,
        horizontal_max_angle=270
    ):
        self.servo_horizontal = ServoController(channel=0, max_angle=270)
        self.servo_vertical   = ServoController(channel=1, max_angle=180)

        self.pid_x = PID(kp=p_horizontal, ki=i_horizontal, kd=d_horizontal,
                         setpoint=0.5, output_limits=(-150, 150))
        self.pid_y = PID(kp=p_vertical, ki=i_vertical, kd=d_vertical,
                         setpoint=0.5, output_limits=(-50, 50))

        self.dead_zone = dead_zone
        self.horizontal_min_angle = horizontal_min_angle
        self.horizontal_max_angle = horizontal_max_angle
        self.vertical_min_angle = vertical_min_angle
        self.vertical_max_angle = vertical_max_angle

    def update_position(self, x_center, y_center):
        """
        Mise à jour des servos en fonction de x_center, y_center
        (normalisés entre 0 et 1).
        """
        # Erreur sur X => angle horizontal
        x_correction = self.pid_x.update(x_center)
        new_horizontal_angle = self.servo_horizontal.current_angle + x_correction
        new_horizontal_angle = max(self.horizontal_min_angle,
                                   min(self.horizontal_max_angle, new_horizontal_angle))

        self.servo_horizontal.set_servo_angle(new_horizontal_angle)

        # Erreur sur Y => angle vertical
        y_correction = self.pid_y.update(y_center)
        new_vertical_angle = self.servo_vertical.current_angle + y_correction
        new_vertical_angle = max(self.vertical_min_angle,
                                 min(self.vertical_max_angle, new_vertical_angle))

        self.servo_vertical.set_servo_angle(new_vertical_angle)

    def position_zero(self):
        """
        Place la caméra à une position de référence.
        """
        self.servo_horizontal.set_servo_angle(135)  # Milieu pour servo 270°
        self.servo_vertical.set_servo_angle(95)     # Milieu approximatif pour servo 180°
        time.sleep(1)

    def position_turn(self):
        self.servo_horizontal.set_servo_angle(0)
        self.servo_vertical.set_servo_angle(45)
        time.sleep(1)
        self.servo_horizontal.set_servo_angle(270)
        self.servo_vertical.set_servo_angle(135)
        time.sleep(1)
        self.position_zero()

    def cleanup_servo(self):
        self.servo_horizontal.cleanup()
        self.servo_vertical.cleanup()
        self.pid_x.reset()
        self.pid_y.reset()


# ========================================
# =============== MAIN ===================
# ========================================
if __name__ == "__main__":
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)

    # Thread pour déplacer la fenêtre vidéo (optionnel)
    window_mover_thread = threading.Thread(target=move_window, daemon=True)
    window_mover_thread.start()
    
    # Initialiser le déplacement de la caméra
    camera_deplacement = CameraDeplacement(
        p_horizontal=30.0,
        i_horizontal=0.01,
        d_horizontal=0.2,
        p_vertical=15.0,
        i_vertical=0.01,
        d_vertical=0.1,
        dead_zone=0.02
    )
    # camera_deplacement.position_turn()
    camera_deplacement.position_zero()
    
    # Récupérer le cairo_overlay pour dessiner
    cairo_overlay = app.pipeline.get_by_name("cairo_overlay")
    if cairo_overlay is None:
        print("Erreur : cairo_overlay non trouvé dans le pipeline.")
        exit(1)
    cairo_overlay.connect("draw", draw_overlay, user_data)

    # On peut lancer en parallèle un thread pour piloter la caméra 
    # en temps quasi-réel (ou on peut le faire directement dans le callback).
    # def camera_control_loop():
    #     while True:
    #         # Utiliser la position filtrée
    #         kx, ky = user_data.kalman_filtered_center
    #         camera_deplacement.update_position(kx, ky)
    #         time.sleep(0.01)  # pour éviter de saturer le CPU
    
    # threading.Thread(target=camera_control_loop, daemon=True).start()
    
    # Lancement de l'application GStreamer
    app.run()
