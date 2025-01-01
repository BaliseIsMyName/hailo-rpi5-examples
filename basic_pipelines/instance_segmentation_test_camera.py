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
        self.detection_event = threading.Event()  # Événement pour signaler les détections
        self.lock = threading.Lock()  # Pour synchroniser l'accès aux détections
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

    # Récupération des détections via Hailo
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    user_data.last_detections = detections
        
    # Signaler qu'une nouvelle détection est disponible
    if detections:
        user_data.detection_event.set()
        
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
    Gère la connexion à un socket Unix local et l'envoi de messages de détection
    uniquement lorsque le service est actif. Intègre la logique d'envoi pour limiter
    la fréquence et éviter les charges CPU excessives.
    """
    def __init__(self, socket_path=SOCKET_PATH, service_name='cameracontrol.service', user_data=None):
        self.socket_path = socket_path
        self.service_name = service_name
        self.user_data = user_data

        self.sock = None
        self.connected = False
        self._stop = False
        self.lock = threading.Lock()  # Pour gérer l'accès concurrent à self.sock

        self.detection_event = threading.Event()

    def start(self):
        """Démarre les threads de vérification de service et de traitement des détections."""
        self.service_thread = threading.Thread(target=self._service_monitor_loop, daemon=True)
        self.service_thread.start()

        self.process_thread = threading.Thread(target=self._process_detections, daemon=True)
        self.process_thread.start()

    def stop(self):
        """Arrête les threads et ferme la socket."""
        self._stop = True
        self.detection_event.set()  # Débloquer la queue si en attente
        with self.lock:
            if self.sock:
                self.sock.close()
        if self.service_thread.is_alive():
            self.service_thread.join()
        if self.process_thread.is_alive():
            self.process_thread.join()

    def _service_monitor_loop(self):
        """Vérifie l'état du service toutes les 10 secondes et gère la connexion."""
        while not self._stop:
            if self._is_service_active():
                if not self.connected:
                    self._connect()
            else:
                if self.connected:
                    self._disconnect()
            time.sleep(10)  # Intervalle de vérification de l'état du service

    def _is_service_active(self):
        """
        Vérifie si le service systemd utilisateur est actif.
        
        Returns:
            bool: True si le service est actif, False sinon.
        """
        try:
            result = subprocess.run(
                ['systemctl', '--user', 'is-active', self.service_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True,
                text=True
            )
            is_active = result.stdout.strip() == 'active'
            if is_active:
                print(f"[UnixDomainSocketSender] Service {self.service_name} est actif.")
            else:
                print(f"[UnixDomainSocketSender] Service {self.service_name} n'est pas actif.")
            return is_active
        except subprocess.CalledProcessError:
            print(f"[UnixDomainSocketSender] Service {self.service_name} n'est pas actif (appel systemctl échoué).")
            return False

    def _connect(self):
        """Tente de se connecter au socket Unix."""
        try:
            with self.lock:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(self.socket_path)
            self.connected = True
            print("[UnixDomainSocketSender] Connecté au serveur camera.")
        except socket.error as e:
            # print(f"[UnixDomainSocketSender] Erreur de connexion : {e}. Retry dans 10s...")
            self.connected = False
            if self.sock:
                self.sock.close()
                self.sock = None

    def _disconnect(self):
        """Déconnecte le socket Unix."""
        with self.lock:
            if self.sock:
                self.sock.close()
                self.sock = None
        self.connected = False
        print("[UnixDomainSocketSender] Déconnecté du serveur camera.")

    def _process_detections(self):
        """Traite et envoie les détections."""
        while not self._stop:
            # Attendre qu'une nouvelle détection soit disponible ou un timeout
            self.detection_event.wait(timeout=0.1)
            self.detection_event.clear()

            if self._stop:
                break

            detections_to_send = []
            with self.user_data.lock:
                detections = list(self.user_data.last_detections)  # Copie sécurisée

            # Filtrer les détections selon le seuil et les objets suivis
            for det in detections:
                label = det.get_label()
                confidence = det.get_confidence()
                if label in TRACK_OBJECTS and confidence >= 0.5:
                    barycentre_x = self.user_data.barycentre_x
                    barycentre_y = self.user_data.barycentre_y
                    if barycentre_x is not None and barycentre_y is not None:
                        detection_msg = {
                            "label": label,
                            "confidence": confidence,
                            "x_center": barycentre_x,
                            "y_center": barycentre_y
                        }
                        detections_to_send.append(detection_msg)

            # Envoyer toutes les détections filtrées
            for detection in detections_to_send:
                self._send_detection(detection)

    def queue_detection(self, detection: dict):
        """Déclenche l'envoi des détections en signalant l'événement."""
        self.detection_event.set()

    def _send_detection(self, detection: dict):
        """Envoie une détection si connecté et service actif."""
        if not self.connected:
            # Ne pas envoyer si non connecté
            return
        
        """
        Envoie un dict JSON représentant la détection:
        {
          "label": "person" ou "cat",
          "confidence": <float>,
          "x_center": <float, 0..1>,
          "y_center": <float, 0..1>
        }
        """
        try:
            message = json.dumps(detection) + "\n"
            with self.lock:
                self.sock.sendall(message.encode("utf-8"))
            print(f"[UnixDomainSocketSender] Détection envoyée: {detection}")
        except socket.error as e:
            print(f"[UnixDomainSocketSender] Erreur d'envoi : {e}")
            self._disconnect()


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

    # 3. Création du Sender (Unix domain socket)
    sender = UnixDomainSocketSender(SOCKET_PATH,
                                    service_name='cameracontrol.service', user_data=user_data)
    sender.start()

    # 5. On récupère le cairo overlay pour dessiner
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
