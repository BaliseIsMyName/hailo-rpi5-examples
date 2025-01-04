#!/usr/bin/env python3
# instance_segmentation_test_camera.py

import os
import socket
import json
import threading
import time
import subprocess

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import hailo
import numpy as np
import matplotlib.pyplot as plt

# Imports PCA9685 & co
import board
import busio
from adafruit_pca9685 import PCA9685

# Imports Hailo RPi communs
from hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)

# Import du pipeline GStreamer instance segmentation
from instance_segmentation_pipeline_modif import GStreamerInstanceSegmentationApp

# =============================
# ========= CONSTANTES ========
# =============================

TRACK_OBJECTS = ["person", "cat"]   # On peut changer si besoin
CONFIDENCE_THRESHOLD = 0.5          # Seuil de confiance min pour le barycentre
DEAD_ZONE = 0.05                    # Zone morte en fraction de l'image (pour l'affichage)

# ========================================
# =========== USER APP CALLBACK ==========
# ========================================

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.last_detections = []
        self.width = None
        self.height = None

        # Liste de barycentres (chacun : (x_norm, y_norm))
        self.barycenters = []

        self.detection_event = threading.Event()
        self.lock = threading.Lock()

        # Zone morte pour l'affichage
        self.dead_zone = DEAD_ZONE


# =============================
# ========== GST CALLBACK =====
# =============================

def app_callback(pad, info, user_data):
    """
    Récupération des détections Hailo, puis calcule TOUS les barycentres
    en utilisant user_data.last_detections.
    """
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    format, width, height = get_caps_from_pad(pad)
    if width is not None and height is not None:
        user_data.width = width
        user_data.height = height

    # Récupération des détections
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    user_data.last_detections = detections

    # Calculer tous les barycentres
    # calc_barycentres(user_data, CONFIDENCE_THRESHOLD)

    # Signaler qu'on a de nouvelles détections / nouveaux barycentres
    user_data.detection_event.set()

    return Gst.PadProbeReturn.OK


# ============================================
# =========== FONCTION calc_barycentres ======
# ============================================
def calc_barycentres(user_data, confidence_threshold=CONFIDENCE_THRESHOLD):
    """
    Parcourt user_data.last_detections pour calculer et stocker DANS user_data.barycenters
    tous les barycentres normalisés (0..1) des masques valides (label suivi, conf >= threshold).
    
    Si pas de détection valide => user_data.barycenters = [] (liste vide).
    """
    # On vide la liste à chaque nouvelle frame
    user_data.barycenters = []

    detections = user_data.last_detections
    if not detections:
        return

    width = user_data.width
    height = user_data.height
    if width is None or height is None:
        return

    # Parcours de toutes les détections
    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        # Vérifier label et confiance
        if label not in TRACK_OBJECTS or confidence < confidence_threshold:
            continue

        # Récupérer le(s) masque(s)
        masks = detection.get_objects_typed(hailo.HAILO_CONF_CLASS_MASK)
        if len(masks) == 0:
            continue

        mask = masks[0]
        mask_height = mask.get_height()
        mask_width = mask.get_width()
        data = np.array(mask.get_data()).reshape((mask_height, mask_width))

        total_weight = np.sum(data)
        if total_weight <= 0:
            continue

        # Indices non nuls
        y_indices, x_indices = np.nonzero(data)
        bary_x_local = np.sum(x_indices * data[y_indices, x_indices]) / total_weight
        bary_y_local = np.sum(y_indices * data[y_indices, x_indices]) / total_weight

        # Récupérer la bbox normalisée
        bbox = detection.get_bbox()

        # Convertir xmin, ymin en pixels
        x_min = int(bbox.xmin() * width)
        y_min = int(bbox.ymin() * height)

        # Suppose qu'il faut multiplier par 4 (downsampling)
        bary_x_pixel = x_min + (bary_x_local * 4)
        bary_y_pixel = y_min + (bary_y_local * 4)

        # Normaliser [0..1]
        bx_norm = round(bary_x_pixel / float(width), 4)
        by_norm = round(bary_y_pixel / float(height), 4)

        user_data.barycenters.append((bx_norm, by_norm))

def draw_overlay(cairooverlay, cr, timestamp, duration, user_data):
    """
    Affiche la zone morte (cercle) + le(s) barycentre(s) (point rouge) si dispo.
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

    # Affichage de TOUS les barycentres stockés
    cr.set_source_rgb(1, 0, 0)  # Rouge
    for (bary_x, bary_y) in user_data.barycenters:
        bx = bary_x * width
        by = bary_y * height

        cr.arc(bx, by, 5, 0, 2 * 3.14159)
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
    Implémentation simple d'un PID (non utilisé pour l'instant).
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
    Pilotage d'un servo via PCA9685 (non utilisé pour l'instant).
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
        # Initialisation du bus I2C + PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=i2c_address)
        self.pca.frequency = freq

        self.channel = channel
        self.servo_min_us = servo_min_us
        self.servo_max_us = servo_max_us
        self.max_angle = max_angle

        # Angle actuel : on suppose qu'on démarre "au milieu"
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
        # Instancie les servos
        self.servo_horizontal = ServoController(channel=0, max_angle=270)
        self.servo_vertical   = ServoController(channel=1, max_angle=180)

        # Instancie les PID
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
        """(Non modifié) - Logique de déplacement selon x_center, y_center."""
        pass

    def position_zero(self):
        """
        Place la caméra à l'angle "zéro" souhaité :
        135° pour l'horizontal et 90° pour le vertical.
        """
        self.servo_horizontal.set_servo_angle(135)
        self.servo_vertical.set_servo_angle(90)
        time.sleep(1)  # Laisser le temps au servo d'atteindre la position

    def cleanup_servo(self):
        """
        Coupe la PWM et reset les PID. 
        (Sans positionner la caméra, désormais fait dans position_zero().)
        """
        self.servo_horizontal.cleanup()
        self.servo_vertical.cleanup()
        self.pid_x.reset()
        self.pid_y.reset()


# ========================================
# =============== MAIN ===================
# ========================================
if __name__ == "__main__":
    Gst.init(None)

    # 1. Préparer l'objet user_data
    user_data = user_app_callback_class()

    # 2. Créer l'application GStreamer
    app = GStreamerInstanceSegmentationApp(app_callback, user_data)

    # 3. Initialiser le déplacement de la caméra
    camera_deplacement = CameraDeplacement(
        p_horizontal=30.0,
        i_horizontal=0.01,
        d_horizontal=0.2,
        p_vertical=15.0,
        i_vertical=0.01,
        d_vertical=0.1,
        dead_zone=0.02
    )

    camera_deplacement.position_zero()
    
    # # 4. Récupérer le cairo_overlay pour dessiner
    # cairo_overlay = app.pipeline.get_by_name("cairo_overlay")
    # if cairo_overlay is None:
    #     print("Erreur : cairo_overlay non trouvé dans le pipeline.")
    #     exit(1)

    # cairo_overlay.connect("draw", draw_overlay, user_data)

    # # Thread pour déplacer la fenêtre vidéo (optionnel)
    # window_mover_thread = threading.Thread(target=move_window, daemon=True)
    # window_mover_thread.start()

    # 5. Lancement de l'application GStreamer
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        camera_deplacement.cleanup_servo()
