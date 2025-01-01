#!/usr/bin/env python3
# detection_main.py

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import os
import socket
import json
import threading
import time
import hailo
import numpy as np
import matplotlib.pyplot as plt
import subprocess


from hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from instance_segmentation_pipeline_modif import GStreamerInstanceSegmentationApp

# -----------------------------
# ParamÃ¨tres de dÃ©tection
# -----------------------------
TRACK_OBJECTS = ["person", "cat"]  # on peut changer si besoin

# -----------------------------
# ParamÃ¨tres du socket
# -----------------------------
SOCKET_PATH = "/tmp/hailo_camera.sock"

# -----------------------------
# CLASS UserData
# -----------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.last_detections = []
        self.width = None
        self.height = None
        self.barycentre_x = None
        self.barycentre_y = None

# -----------------------------
# Callbacks GStreamer
# -----------------------------
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    format, width, height = get_caps_from_pad(pad)
    if width is not None and height is not None:
        user_data.width = width
        user_data.height = height

    # RÃ©cupÃ©ration des dÃ©tections via Hailo
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    user_data.last_detections = detections
        
    return Gst.PadProbeReturn.OK

def draw_overlay(cairooverlay, cr, timestamp, duration, user_data):
    if user_data.width is None or user_data.height is None:
        return

    width = user_data.width
    height = user_data.height

    # Dessiner un point bleu au centre de la vidÃ©o
    cr.set_source_rgb(0, 0, 1)  # Bleu
    cr.arc(width / 2, height / 2, 5, 0, 2 * 3.14159)
    cr.fill()

    # Dessiner un point rouge au barycentre de chaque masque de dÃ©tection
    cr.set_source_rgb(1, 0, 0)  # Rouge
    for det in user_data.last_detections:
        label = det.get_label()  # "person", "cat", etc.
        if label not in TRACK_OBJECTS:
            continue

        bbox = det.get_bbox()
        masks = det.get_objects_typed(hailo.HAILO_CONF_CLASS_MASK)
        if len(masks) != 0:
            mask = masks[0]
            mask_height = mask.get_height()
            mask_width = mask.get_width()
            data = np.array(mask.get_data())
            data = data.reshape((mask_height, mask_width))

            # Calculer le barycentre du masque
            y_indices, x_indices = np.nonzero(data)
            total_weight = np.sum(data)
            if total_weight > 0:
                barycentre_x = int(np.sum(x_indices * data[y_indices, x_indices]) / total_weight)
                barycentre_y = int(np.sum(y_indices * data[y_indices, x_indices]) / total_weight)

                # Convertir les coordonnÃ©es du barycentre en coordonnÃ©es de l'image
                x_min, y_min = int(bbox.xmin() * width), int(bbox.ymin() * height)
                barycentre_x = x_min + barycentre_x * 4
                barycentre_y = y_min + barycentre_y * 4

                # Stocker les coordonnÃ©es du barycentre dans user_data
                user_data.barycentre_x = barycentre_x / width
                user_data.barycentre_y = barycentre_y / height
                
                # Dessiner un point au barycentre sur l'image
                cr.arc(barycentre_x, barycentre_y, 5, 0, 2 * 3.14159)
                cr.fill()

# -----------------------------
# Classe Sender pour Unix Domain Socket
# -----------------------------
class UnixDomainSocketSender:
    """
    Gère la connexion à un socket Unix local et l'envoi d'un seul message
    (type dict) à la fois.
    """
    def __init__(self, socket_path=SOCKET_PATH):
        self.socket_path = socket_path
        self.sock = None
        self.connected = False
        self._stop = False
        self.lock = threading.Lock()  # Pour gérer l'accès concurrent à self.sock
        
    def start(self):
        """Essaye de se connecter au socket. Peut être relancé si le serveur n'est pas prêt."""
        self.thread = threading.Thread(target=self._connect_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Arrête le thread de connexion et ferme la socket."""
        self._stop = True
        with self.lock:
            if self.sock:
                self.sock.close()
        if self.thread.is_alive():
            self.thread.join()

    def _connect_loop(self):
        while not self._stop:
            if not self.connected:
                try:
                    with self.lock:
                        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        self.sock.connect(self.socket_path)
                    self.connected = True
                    print("[UnixDomainSocketSender] Connecté au serveur camera.")
                except socket.error as e:
                    print(f"[UnixDomainSocketSender] Erreur de connexion : {e}. Retry dans 2s...")
                    time.sleep(2)
                    continue  # Recommencer la boucle pour tenter de se reconnecter

            # Vérifier si la connexion est toujours active
            try:
                # Envoyer un ping ou vérifier la socket
                self.sock.sendall(b'')  # Envoi d'une chaîne vide pour vérifier la connexion
            except socket.error:
                print("[UnixDomainSocketSender] Déconnecté du serveur camera.")
                self.connected = False
                with self.lock:
                    self.sock.close()
                time.sleep(2)  # Attendre avant de tenter de se reconnecter

            time.sleep(1)  # Intervalle avant la prochaine vérification

    def send_detection(self, detection: dict):
        """
        Envoie un dict JSON représentant la détection:
        {
          "label": "person" ou "cat",
          "confidence": <float>,
          "x_center": <float, 0..1>,
          "y_center": <float, 0..1>
        }
        """
        if not self.connected or not self.sock:
            print("[UnixDomainSocketSender] Non connecté. Détection non envoyée.")
            return

        try:
            message = json.dumps(detection) + "\n"
            with self.lock:
                self.sock.sendall(message.encode("utf-8"))
            print(f"[UnixDomainSocketSender] Détection envoyée: {detection}")
        except socket.error as e:
            print(f"[UnixDomainSocketSender] Erreur d'envoi : {e}")
            self.connected = False
            with self.lock:
                self.sock.close()
            # La connexion sera réessayée dans le _connect_loop

# -----------------------------déplacement de la fenêtre dans le coin en haut à droite de l'écran -----------------------------
def move_window():
    while True:
        try:
            window_ids = subprocess.check_output(['xdotool', 'search', '--name', 'Hailo Detection App']).decode().strip().split('\n')
            if window_ids:
                window_id = window_ids[0]
                subprocess.run(['xdotool', 'windowmove', window_id, '440', '62'])
                print(f"Fenêtre déplacée : ID {window_id} vers (440, 62)")
                break  # Terminer le thread après déplacement
            else:
                print("Fenêtre 'Hailo Detection App' non trouvée, tentative suivante...")
        except subprocess.CalledProcessError:
            print("Erreur lors de la recherche de la fenêtre 'Hailo Detection App', tentative suivante...")
        time.sleep(1)


# -----------------------------
# MAIN
# -----------------------------
if __name__ == "__main__":
    Gst.init(None)

    # 1. PrÃ©parer l'objet user_data
    user_data = user_app_callback_class()

    # 2. CrÃ©ation de l'application GStreamer
    app = GStreamerInstanceSegmentationApp(app_callback, user_data)

    # 3. CrÃ©ation du Sender (Unix domain socket)
    sender = UnixDomainSocketSender(SOCKET_PATH)
    sender.start()

    # 4. Lancement d'un thread qui choisit la dÃ©tection la plus confiante
    def send_loop():
        while True:
            try:
                best_confidence = 0.0
                best_det = None
                if user_data.last_detections:
                    for det in user_data.last_detections:
                        label = det.get_label()
                        if label not in TRACK_OBJECTS:
                            continue
                        confidence = det.get_confidence()
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_det = det

                if best_det and user_data.barycentre_x is not None and user_data.barycentre_y is not None:
                    detection_msg = {
                        "label": best_det.get_label(),
                        "confidence": best_confidence,
                        "x_center": user_data.barycentre_x,
                        "y_center": user_data.barycentre_y
                    }
                    sender.send_detection(detection_msg)
            except Exception as e:
                print(f"[send_loop] Erreur lors de l'envoi des données : {e}")

            time.sleep(0.1)  # 10 fois par seconde

    t = threading.Thread(target=send_loop, daemon=True)
    t.start()

    # 5. On rÃ©cupÃ¨re le cairo overlay pour dessiner
    cairo_overlay = app.pipeline.get_by_name("cairo_overlay")
    if cairo_overlay is None:
        print("Erreur : cairo_overlay non trouvé dans le pipeline.")
        exit(1)

    cairo_overlay.connect("draw", draw_overlay, user_data)

        # Lancer un thread pour déplacer la fenêtre vidéo
    window_mover_thread = threading.Thread(target=move_window, daemon=True)
    window_mover_thread.start()
    
    # 6. Lancement de l'appli GStreamer
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()
