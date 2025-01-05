#detection.py

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
import cairo
import signal
import sys

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
from deepsort_tracker import DeepSORTTracker  # Importing the DeepSORT tracker

# =============================
# ========= CONSTANTES ========
# =============================

TRACK_OBJECTS = ["person", "cat"]   # Labels à suivre
CONFIDENCE_THRESHOLD = 0.5          # Seuil de confiance min
DEAD_ZONE = 0.05                    # Zone morte (en fraction de l'image)
# Flag pour indiquer si l'application doit s'arrêter
should_exit = False

def signal_handler(signum, frame):
    global should_exit
    print(f"Signal {signum} reçu. Fermeture de l'application...")
    should_exit = True
    quit_app(should_exit)
    
def quit_app(should_exit):
    if should_exit:
        print("Arrêt de l'application...")
        user_data.camera_controller.stop()
        user_data.camera_controller.camera_deplacement.position_zero()
        user_data.camera_controller.camera_deplacement.cleanup_servo()
        app.shutdown()  # Arrêter le GLib.MainLoop

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

        # === DEEPSORT TRACKER ===
        self.tracker = DeepSORTTracker()  # Initialize the DeepSORT tracker

        # === TRACK SELECTOR ===
        self.track_selector = TrackSelector()

        # === CAMERA CONTROLLER ===
        self.camera_controller = CameraController(camera_deplacement)
        
        self.current_fps = 0.0
        self.current_droprate = 0.0
        self.avg_fps = 0.0
        
        self.detection_event = threading.Event()
        self.lock = threading.Lock()

        self.dead_zone = DEAD_ZONE


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
    
    # Mise à jour de la liste des centres
    calc_bbox_centers(user_data, CONFIDENCE_THRESHOLD)

    # === DEEPSORT TRACKER ===
    # Convert detections to a format suitable for DeepSORT
    detection_bboxes = []
    for d in detections:
        if d.get_label() in TRACK_OBJECTS and d.get_confidence() >= CONFIDENCE_THRESHOLD:
            bbox = d.get_bbox()
            detection_bboxes.append([bbox.xmin(), bbox.ymin(), bbox.xmax(), bbox.ymax()])

    # Update the tracker with the current detections
    user_data.tracker.update(detection_bboxes)

    # Retrieve the current tracks
    tracks = user_data.tracker.get_tracks()

    # Mettre à jour le TrackSelector
    user_data.track_selector.update_tracks(tracks)

    # Obtenir le centre du track sélectionné
    selected_center = user_data.track_selector.get_selected_track_center()
    if selected_center:
        user_data.camera_controller.update_center(selected_center)


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

    cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(14)
    # === DEEPSORT TRACKER ===
    # Dessin des tracks en vert
    cr.set_source_rgb(0, 1, 0)  # Vert
    for track in user_data.tracker.get_tracks():
        center_x, center_y = track.center
        # print(f"[DISPLAY] Track ID={track.track_id}, center_x={round(center_x, 4)},center_y={round(center_y, 4)}")
        bx_f = center_x * width
        by_f = center_y * height
        cr.arc(bx_f, by_f, 4, 0, 2 * 3.14159)
        cr.fill()

        # Draw track ID above the bounding box
        track_id = str(track.track_id)
        # print(f"Track ID {track_id} : {track.bbox}")
        cr.move_to(bx_f + 10 , by_f + 10)  # Position text above the bounding box
        cr.show_text(track_id)

# =============================
# ======= TRACK SELECTOR ======
# =============================
class TrackSelector:
    """
    Sélectionne quel track suivre en fonction des critères définis.
    Par exemple, suit le track avec l'ID le plus ancien.
    """
    def __init__(self):
        self.tracks = []
        self.selected_track_id = None
        self.lock = threading.Lock()

    def update_tracks(self, tracks):
        with self.lock:
            self.tracks = tracks
            self.select_track()

    def select_track(self):
        """
        Sélectionne le track à suivre. Ici, on choisit le track avec l'ID le plus bas (le plus ancien).
        """
        if not self.tracks:
            self.selected_track_id = None
            return

        # Trier les tracks par ID croissant (supposant que les IDs augmentent avec le temps)
        sorted_tracks = sorted(self.tracks, key=lambda t: t.track_id)
        self.selected_track_id = sorted_tracks[0].track_id

    def get_selected_track_center(self):
        """
        Retourne le centre normalisé du track sélectionné.
        """
        with self.lock:
            if self.selected_track_id is None:
                return None

            for track in self.tracks:
                if track.track_id == self.selected_track_id:
                    return track.center
            return None

# ========================================
# =========== PILOTAGE CAMERA ============
# ========================================
class PID:
    def __init__(self, 
                 kp=1.0, 
                 ki=0.0, 
                 kd=0.0, 
                 setpoint=0.5, 
                 output_limits=(-999, 999)
                 ):
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

# ===================================
# ======= CAMERA SERVOCONTROLLER ========
# ===================================
class ServoController:
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

# ===================================
# ======= CAMERA DEPLACEMENT ========
# ===================================
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

# ===================================
# ======= CAMERA CONTROLLER ========
# ===================================
class CameraController:
    """
    Pilote la caméra en fonction des coordonnées fournies.
    """
    def __init__(self, camera_deplacement):
        self.camera_deplacement = camera_deplacement
        self.current_center = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def update_center(self, center):
        with self.lock:
            self.current_center = center

    def run(self):
        while self.running:
            with self.lock:
                center = self.current_center
            if center:
                x_center, y_center = center
                # Vérifier si l'objet est en dehors de la zone morte
                if (abs(x_center - 0.5) > self.camera_deplacement.dead_zone or
                    abs(y_center - 0.5) > self.camera_deplacement.dead_zone):
                    self.camera_deplacement.update_position(x_center, y_center)
            time.sleep(0.1)  # Ajuster la fréquence selon les besoins

    def stop(self):
        self.running = False
        self.thread.join()

# ===================================
# ======= DEPLACEMENT WINDOWS APP ========
# ===================================
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
# =============== MAIN ===================
# ========================================
if __name__ == "__main__":
    # Enregistrer les gestionnaires de signaux
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
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
        dead_zone=0.05
    )
    # camera_deplacement.position_turn()
    camera_deplacement.position_zero()
    
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    
    # Initialiser le TrackSelector et le CameraController
    user_data.track_selector = TrackSelector()
    user_data.camera_controller = CameraController(camera_deplacement)
    
    # Récupérer le cairo_overlay pour dessiner
    cairo_overlay = app.pipeline.get_by_name("cairo_overlay")
    if cairo_overlay is None:
        print("Erreur : cairo_overlay non trouvé dans le pipeline.")
        exit(1)
    cairo_overlay.connect("draw", draw_overlay, user_data)

    # Lancement de l'application GStreamer
    try:
        app.run()  # Démarrer le GLib.MainLoop
    except Exception as e:
        print(f"Exception rencontrée : {e}")
    finally:
        print("Application fermée proprement.")

#Fin de detection.py