# detection.py

import signal
import sys
import threading
import time
import subprocess
from typing import Any, List, Optional, Tuple
import yaml
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import shutil
from datetime import datetime
import re
from collections import defaultdict

import board
import busio
from adafruit_pca9685 import PCA9685

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import cairo
import hailo

from hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from detection_pipeline import GStreamerDetectionApp
from deepsort_tracker import DeepSORTTracker

from notifBot import send_telegram_message

import logging
from dataclasses import dataclass, field

# Configuration du logger
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Configuration spécifique pour ServoController
servo_logger = logging.getLogger('ServoController')
servo_logger.setLevel(logging.INFO)  # Activer DEBUG uniquement pour ServoController

# Configuration spécifique pour PID
pid_logger = logging.getLogger('PID')
pid_logger.setLevel(logging.INFO)  # Activer DEBUG uniquement pour PID

# =============================
# ========= CONSTANTES ========
# =============================

# Couleurs utilisées dans l'overlay
COLOR_BLUE: Tuple[float, float, float] = (0, 0, 1)
COLOR_RED: Tuple[float, float, float] = (1, 0, 0)
COLOR_GREEN: Tuple[float, float, float] = (0, 1, 0)
COLOR_YELLOW: Tuple[float, float, float] = (1, 1, 0)

@dataclass
class DetectionAppState:
    should_exit: bool = False
    user_data: Any = field(default=None)

# ========================================
# =========== USER APP CALLBACK ==========
# ========================================
@dataclass
class UserAppCallback(app_callback_class):
    """
    Classe de rappel utilisateur pour gérer les détections et le suivi.
    """
    detection_app: Optional['DetectionApp'] = None  # Référence à DetectionApp

    def __init__(self) -> None:
        super().__init__()
        self.last_detections: List[hailo.HAILO_DETECTION] = []
        self.width: Optional[int] = None
        self.height: Optional[int] = None
        self.bbox_centers: List[Tuple[float, float]] = []
        self.tracker: DeepSORTTracker = DeepSORTTracker()
        self.track_selector: TrackSelector = TrackSelector()
        self.camera_controller: Optional[CameraController] = None  # Initialisé dans le main
        self.current_fps: float = 0.0
        self.current_droprate: float = 0.0
        self.avg_fps: float = 0.0
        self.detection_event: threading.Event = threading.Event()
        self.lock: threading.Lock = threading.Lock()
        self.dead_zone: float = DEAD_ZONE
  
  
class HlsSegmentRecorder:
    """
    Gère la copie des segments HLS dans un dossier temporaire
    et la création d'une vidéo finale quand la détection se termine.
    """

    def __init__(self,
                 hls_folder: str = "/var/www/html/hailo_app/stream/hls",
                 tmp_folder: str = "/tmp/hls_segments",
                 final_folder: str = "/home/raspi/Videos/records",
                 segment_extension: str = ".ts",
                 poll_interval: float = 1.0):
        """
        Args:
            hls_folder:        Dossier où sont générés les segments HLS.
            tmp_folder:        Dossier temporaire où on recopie les segments utiles.
            final_folder:      Dossier final où on range la vidéo reconstituée.
            segment_extension: Extension des segments HLS (".ts", ".m4s", etc.).
            poll_interval:     Intervalle en secondes pour scruter le dossier HLS.
        """
        self.hls_folder = hls_folder
        self.tmp_folder = tmp_folder
        self.final_folder = final_folder
        self.segment_extension = segment_extension
        self.poll_interval = poll_interval

        # Définir les seuils de frames par classe directement dans le code
        self.frame_thresholds = {
            "person": 20,
            "cat": 30  # Assurez-vous que cela correspond à CameraController
        }
        
        self.valid_detection_frame_count = 0  # Initialiser le compteur
        self.detection_active_labels = set()
        self.copied_segments = set()  # Segments copiés par classe
        self.start_times = defaultdict(datetime)
    
        # État
        self.detection_active = False
        self.running = True
        self.lock = threading.Lock()

        # Thread de scrutation
        self.thread = threading.Thread(target=self._copy_loop, daemon=True)
        self.thread.start()

    def start_detection(self):
        """
        Démarre la copie des segments lorsqu'une détection valide commence.
        """
        with self.lock:
            if not self.detection_active:
                self.detection_active = True
                self.copied_segments.clear()
                logger.info("[HlsSegmentRecorder] Détection démarrée. Commencement de la copie des segments.")

    @staticmethod        
    def extract_segment_number(path: str) -> int:
        """
        Extrait la partie numérique d'un nom de fichier de type 'stream-298.ts'.
        Retourne 999999999 si rien n'est trouvé, pour éviter les crash.
        """
        filename = os.path.basename(path)  # ex: "stream-298.ts"
        m = re.search(r'stream-(\d+)\.ts$', filename)
        if m:
            return int(m.group(1))
        else:
            return 999999999
    
    def stop_detection_and_finalize(self, valid_frame_counts: dict):
        """
        Arrête la copie des segments, vérifie les seuils de frames valides par objet,
        assemble les segments copiés en une vidéo si les seuils sont atteints,
        renomme le fichier final avec les labels des objets détectés,
        puis déplace la vidéo dans final_folder.
        Sinon, supprime les segments copiés.
        
        Args:
            valid_frame_counts (dict): Dictionnaire avec le nombre de frames valides par objet.
        """
        with self.lock:
            if self.detection_active:
                self.detection_active = False
                logger.info("[HlsSegmentRecorder] Fin de détection. Arrêt de la copie des segments.")

                # Vérifier si tous les objets atteignent le seuil de frames
                objects_valid = [label for label, count in valid_frame_counts.items()
                                 if count >= self.frame_thresholds.get(label, 30)]
                
                if objects_valid:
                    logger.info(f"Enregistrement validé pour les objets : {', '.join(objects_valid)}")
                    # Assembler les segments en une vidéo finale
                    self._assemble_segments(objects_valid)
                else:
                    logger.info("Enregistrement invalidé : seuil de frames non atteint pour les objets détectés.")
                    # Nettoyer les segments temporaires sans créer de fichier final
                    self._cleanup_segments()

    def _assemble_segments(self, objects_valid: List[str]):
        """
        Concatène les segments copiés en une seule vidéo et déplace le fichier final.

        Args:
            objects_valid (List[str]): Liste des labels des objets validés.
        """
        if not self.copied_segments:
            logger.warning("[HlsSegmentRecorder] Aucun segment à assembler.")
            return

        # Trier les segments par numéro
        ts_files_sorted = sorted(
            self.copied_segments,
            key=lambda x: int(re.search(r'stream-(\d+)\.ts$', x).group(1)) if re.search(r'stream-(\d+)\.ts$', x) else 0
        )

        segment_fullpaths = [os.path.join(self.tmp_folder, f) for f in ts_files_sorted]

        # Créer un fichier "mylist.txt" pour ffmpeg
        list_path = os.path.join(self.tmp_folder, "mylist.txt")
        with open(list_path, "w") as f:
            for seg in segment_fullpaths:
                f.write(f"file '{seg}'\n")

        # Définir le nom final de la vidéo avec les labels des objets
        objects_str = '+'.join(objects_valid)
        now_str = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        final_name = f"{objects_str}_{now_str}.mp4"
        final_path = os.path.join(self.final_folder, final_name)

        # Exécuter ffmpeg pour concaténer les segments
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "concat", "-safe", "0",
            "-i", list_path,
            "-c", "copy",
            final_path
        ]
        logger.info(f"[HlsSegmentRecorder] Assemblage final avec ffmpeg : {' '.join(cmd)}")
        try:
            subprocess.run(cmd, check=True)
            logger.info(f"[HlsSegmentRecorder] Vidéo finale créée : {final_path}")

            # Envoyer une notification Telegram
            base_url = "www.boissard.net/records/video/"
            video_url = f"{base_url}{final_name}"
            message = f"Une nouvelle vidéo a été enregistrée : {video_url}"
            send_telegram_message(message)

            # Nettoyer les segments temporaires et le fichier de liste
            for seg in ts_files_sorted:
                try:
                    os.remove(os.path.join(self.tmp_folder, seg))
                    logger.info(f"[HlsSegmentRecorder] Segment {seg} supprimé après assemblage.")
                except OSError:
                    logger.error(f"[HlsSegmentRecorder] Erreur lors de la suppression du segment {seg}.")

            try:
                os.remove(list_path)
                logger.info(f"[HlsSegmentRecorder] Fichier de liste {list_path} supprimé.")
            except OSError:
                logger.error(f"[HlsSegmentRecorder] Erreur lors de la suppression du fichier de liste {list_path}.")

        except subprocess.CalledProcessError as e:
            logger.error(f"[HlsSegmentRecorder] Erreur lors de l'assemblage des segments avec ffmpeg : {e}")

    def stop_all(self):
        """
        Appelée lors de la fermeture de l'application pour terminer proprement.
        """
        self.running = False
        self.thread.join()
        print("[HlsSegmentRecorder] Thread de copie arrêté.")

    def _copy_loop(self):
        while self.running:
            time.sleep(self.poll_interval)

            with self.lock:
                if not self.detection_active:
                    continue

                try:
                    files = os.listdir(self.hls_folder)
                    logger.debug(f"[HlsSegmentRecorder] Fichiers dans hls_folder: {files}")
                except FileNotFoundError:
                    logger.error(f"[HlsSegmentRecorder] Dossier HLS non trouvé: {self.hls_folder}")
                    continue

                ts_files = sorted([
                    f for f in files if f.endswith(self.segment_extension)
                ])

                for fname in ts_files:
                    if fname not in self.copied_segments:
                        src_path = os.path.join(self.hls_folder, fname)
                        dst_path = os.path.join(self.tmp_folder, fname)

                        # Vérifier la stabilité du fichier
                        if self._file_is_stable(src_path):
                            os.makedirs(self.tmp_folder, exist_ok=True)
                            try:
                                shutil.copy2(src_path, dst_path)
                                self.copied_segments.add(fname)
                                logger.debug(f"[HlsSegmentRecorder] Segment {fname} copié.")
                            except Exception as e:
                                logger.error(f"[HlsSegmentRecorder] Erreur copie {fname}: {e}")


    def _cleanup_segments(self):
        """
        Supprime tous les segments copiés sans créer de fichier final.
        """
        for seg in self.copied_segments.copy():
            dst_path = os.path.join(self.tmp_folder, seg)
            try:
                os.remove(dst_path)
                logger.info(f"[HlsSegmentRecorder] Segment {seg} supprimé (enregistrement invalidé).")
                self.copied_segments.remove(seg)
            except OSError:
                logger.error(f"[HlsSegmentRecorder] Erreur lors de la suppression du segment {seg}.")


    def _file_is_stable(self, filepath, check_delay=0.5):
        """Vérifie que la taille du fichier ne change plus sur check_delay secondes."""
        try:
            size1 = os.path.getsize(filepath)
            time.sleep(check_delay)
            size2 = os.path.getsize(filepath)
            return (size1 == size2)
        except OSError:
            return False
    
    @staticmethod
    def cleanup_old_videos(records_folder: str = "/home/raspi/Videos/records"):
        """
        Conserve toutes les vidéos du jour.
        Conserve ensuite suffisamment de vidéos "anciennes" pour ne pas dépasser 10 au total.
        Supprime le reste des vidéos les plus anciennes.
        """
        if not os.path.exists(records_folder):
            return

        # Lister tous les fichiers .mp4 (ou .mkv, etc.) dans records_folder
        all_files = [f for f in os.listdir(records_folder) if f.endswith(".mp4")]
        if not all_files:
            return

        # Créer une structure (filepath complet + date extraite + ctime).
        # On suppose que le nom contient "YYYY-mm-dd_HH-MM"
        today_str = datetime.now().strftime("%Y-%m-%d")  # ex "2025-01-26"

        videos_info = []
        for fname in all_files:
            fullpath = os.path.join(records_folder, fname)
            
            # 1) Extraire la date du nom : "cat+person_2025-01-26_16-42.mp4"
            #    On cherche un pattern du type: _YYYY-MM-DD_HH-MM
            #    Pas à pas ou via regex
            m = re.search(r'_(\d{4}-\d{2}-\d{2})_(\d{2}:\d{2}:\d{2})\.mp4$', fname)
            if m:
                file_date_str = m.group(1)  # ex "2025-01-26"
                # file_time_str = m.group(2) # ex "16-42" (pas forcément utile)
            else:
                file_date_str = "9999-99-99"  # si le pattern n'est pas trouvé
            # On stocke (fullpath, is_today, ctime, filename)
            st = os.stat(fullpath)
            ctime = st.st_ctime  # en secondes
            is_today = (file_date_str == today_str)

            videos_info.append({
                "path": fullpath,
                "filename": fname,
                "date_str": file_date_str,
                "is_today": is_today,
                "ctime": ctime
            })

        # Séparer en 2 groupes : les vidéos "d'aujourd'hui" et les "anciennes"
        todays = [v for v in videos_info if v["is_today"]]
        olds   = [v for v in videos_info if not v["is_today"]]

        # On conserve toutes les vidéos "aujourd'hui"
        # On veut un total final <= 10, sauf si today's alone est déjà > 10 => on s'en fiche, on conserve tout
        # => On calcule combien de slots il reste pour les "olds"
        nb_today = len(todays)
        if nb_today >= 10:
            # On ne supprime aucune d'aujourd'hui, même si c'est au-delà de 10
            # => On supprime toutes les olds
            for vid in olds:
                try:
                    os.remove(vid["path"])
                    print(f"[cleanup_old_videos] Removed old video {vid['filename']}")
                except:
                    pass
            return
        else:
            # nb_today < 10 => on peut garder un certain nombre d'old
            # => on trie olds par ctime desc (plus récent en premier)
            # => on garde les plus récents tant qu'on n'atteint pas 10 total
            olds_sorted = sorted(olds, key=lambda v: v["ctime"], reverse=True)  # plus récent d'abord
            # combien on peut garder d'olds ?
            slots_available = 10 - nb_today
            keep_olds = olds_sorted[:slots_available]   # on garde les plus récents
            remove_olds = olds_sorted[slots_available:] # on supprime le reste

            for vid in remove_olds:
                try:
                    os.remove(vid["path"])
                    print(f"[cleanup_old_videos] Removed old video {vid['filename']}")
                except:
                    pass

        print("[cleanup_old_videos] Cleanup done.")

# =============================
# ======= TRACK SELECTOR ======
# =============================
class TrackSelector:
    """
    Sélectionne quel track suivre en fonction des critères définis.
    Par exemple, suit le track avec l'ID le plus ancien.
    """

    def __init__(self) -> None:
        self.tracks: List[Any] = []
        self.selected_track_id: Optional[int] = None
        self.lock: threading.Lock = threading.Lock()

    def update_tracks(self, tracks: List[Any]) -> None:
        with self.lock:
            self.tracks = tracks
            self.select_track()

    def select_track(self) -> None:
        """
        Sélectionne le track à suivre. Ici, on choisit le track avec l'ID le plus bas (le plus ancien).
        """
        if not self.tracks:
            self.selected_track_id = None
            logger.debug("[TrackSelector] Aucun track actif trouvé.")
            return

        # Trier les tracks par ID croissant
        sorted_tracks = sorted(self.tracks, key=lambda t: t.track_id)
        self.selected_track_id = sorted_tracks[0].track_id
        logger.debug(f"[TrackSelector] Track actif sélectionné ID={self.selected_track_id}")

    def get_selected_track_info(self) -> Tuple[Optional[Tuple[float, float]], Optional[int]]:
        """
        Retourne le centre normalisé du track sélectionné.
        """
        with self.lock:
            if self.selected_track_id is None:
                logger.debug("[TrackSelector] Aucun track sélectionné.")
                return None, None

            for track in self.tracks:
                if track.track_id == self.selected_track_id:
                    return track.center, track.time_since_update

            logger.debug("[TrackSelector] Track sélectionné non trouvé dans les tracks actuels.")
            return None, None

# ===================================
# ========= PILOTAGE CAMERA =========
# ===================================
class PID:
    """
    Contrôleur PID simple.
    """
    def __init__(
        self, 
        kp: float = 1.0, 
        ki: float = 0.0, 
        kd: float = 0.0, 
        setpoint: float = 0.5, 
        output_limits: Tuple[float, float] = (-999, 999),
        epsilon: float = 0.01,  # Tolérance pour les changements significatifs
    ) -> None:
        self.logger = logging.getLogger('PID')
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.setpoint: float = setpoint
        self.output_limits: Tuple[float, float] = output_limits
        self._integral: float = 0.0
        self._last_error: float = 0.0
        self._last_time: float = time.time()

        # Attributs pour le suivi des dernières valeurs logguées
        self.last_logged_error: Optional[float] = None
        self.last_logged_p: Optional[float] = None
        self.last_logged_i: Optional[float] = None
        self.last_logged_d: Optional[float] = None
        self.last_logged_output: Optional[float] = None

        # Tolérance pour déterminer les changements significatifs
        self.epsilon: float = epsilon

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def update(self, measurement: float) -> float:
        now: float = time.time()
        dt: float = now - self._last_time
        if dt <= 0.0:
            dt = 1e-16

        error: float = self.setpoint - measurement
        p_out: float = self.kp * error
        self._integral += error * dt
        i_out: float = self.ki * self._integral
        derivative: float = (error - self._last_error) / dt
        d_out: float = self.kd * derivative

        output: float = p_out + i_out + d_out
        min_out, max_out = self.output_limits
        output = max(min_out, min(output, max_out))

        self._last_error = error
        self._last_time = now

        # Vérifier si les changements sont significatifs avant de loguer
        if self.is_significant_change(error, p_out, i_out, d_out, output):
            # Log détaillé des composants PID
            self.logger.debug(
                f"PID Update -> Error: {error:.4f}, "
                f"P: {p_out:.4f}, I: {i_out:.4f}, D: {d_out:.4f}, "
                f"Output: {output:.4f}"
            )
            # Mettre à jour les dernières valeurs logguées
            self.last_logged_error = error
            self.last_logged_p = p_out
            self.last_logged_i = i_out
            self.last_logged_d = d_out
            self.last_logged_output = output

        return output

    def is_significant_change(
            self, 
            error: float, 
            p_out: float, 
            i_out: float, 
            d_out: float, 
            output: float
        ) -> bool:
        """
        Détermine si les changements actuels sont significatifs par rapport aux dernières valeurs logguées.
        """
        if self.last_logged_error is None:
            return True  # Toujours loguer la première occurrence

        return (
            abs(error - self.last_logged_error) >= self.epsilon or
            abs(p_out - self.last_logged_p) >= self.epsilon or
            abs(i_out - self.last_logged_i) >= self.epsilon or
            abs(d_out - self.last_logged_d) >= self.epsilon or
            abs(output - self.last_logged_output) >= self.epsilon
        )

# ===================================
# ======= CAMERA SERVOCONTROLLER =====
# ===================================
class ServoController:
    """
    Contrôleur pour un servo-moteur via PCA9685.
    """

    def __init__(
        self,
        channel: int = 0,
        freq: int = 50,
        i2c_address: int = 0x40,
        servo_min_us: int = 500,
        servo_max_us: int = 2500,
        max_angle: int = 180
    ) -> None:
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca: PCA9685 = PCA9685(i2c, address=i2c_address)
        self.pca.frequency = freq
        self.logger = logging.getLogger('ServoController')
        self.previous_angle: Optional[float] = None
        self.max_angle: int = max_angle
        self.servo_min_us: int = servo_min_us
        self.servo_max_us: int = servo_max_us
        self.current_angle: float = max_angle / 2.0
        self.channel: int = channel
        self.set_servo_angle(self.current_angle)
        
    def _us_to_duty_cycle(self, pulse_us: int) -> int:
        period_us: int = 1_000_000 // self.pca.frequency  # ex: 20_000µs @ 50Hz
        duty_cycle: int = int((pulse_us / period_us) * 65535)
        return max(0, min(65535, duty_cycle))

    def set_servo_angle(self, angle_deg: float) -> None:
        angle_clamped: float = max(0, min(self.max_angle, angle_deg))
        
        # Définir une tolérance pour les petites variations
        epsilon: float = 0.1  # Ajustez cette valeur selon vos besoins

        # Vérifier si l'angle a réellement changé de manière significative
        if self.previous_angle is not None:
            angle_difference = abs(angle_clamped - self.previous_angle)
            if angle_difference < epsilon:
                # Pas de changement significatif, ne pas enregistrer le log
                return

        self.current_angle = angle_clamped

        span_us: int = self.servo_max_us - self.servo_min_us
        pulse_us: float = self.servo_min_us + (span_us * (angle_clamped / float(self.max_angle)))
        self.pca.channels[self.channel].duty_cycle = self._us_to_duty_cycle(int(pulse_us))

        # Enregistrer le log seulement si l'angle a changé de manière significative
        self.logger.debug(f"Servo Channel {self.channel} Angle set to {self.current_angle}° (Pulse: {pulse_us}µs)")

        # Mettre à jour l'angle précédent
        self.previous_angle = angle_clamped

    def cleanup(self) -> None:
        # self.pca.channels[self.channel].duty_cycle = 0
        self.pca.deinit()
        self.logger.info(f"Servo Channel {self.channel} cleanup completed.")

# ===================================
# ======= CAMERA DEPLACEMENT ========
# ===================================
class CameraDeplacement:
    """
    Gère deux servos (horizontal + vertical) avec 2 PID.
    """

    def __init__(
        self,
        pid_config: dict,
        servo_config: dict,
        camera_movement_config: dict,
        fader_config: dict = None
    ) -> None:
        
        # Initialisation des servos
        self.servo_horizontal: ServoController = ServoController(
            channel=servo_config.get("channel_horizontal", 0),
            max_angle=servo_config.get("max_angle_horizontal", 270),
            freq=servo_config.get("freq", 50),
            i2c_address=servo_config.get("i2c_address", 0x40),
            servo_min_us=servo_config.get("servo_min_us", 500),
            servo_max_us=servo_config.get("servo_max_us", 2500)
        )
        self.servo_vertical: ServoController = ServoController(
            channel=servo_config.get("channel_vertical", 1),
            max_angle=servo_config.get("max_angle_vertical", 180),
            freq=servo_config.get("freq", 50),
            i2c_address=servo_config.get("i2c_address", 0x40),
            servo_min_us=servo_config.get("servo_min_us", 500),
            servo_max_us=servo_config.get("servo_max_us", 2500)
        )

        # Stockage des paramètres PID pour objets grands et petits
        self.pid_config = pid_config

        # Initialisation des contrôleurs PID
        self.pid_x: PID = PID(
            kp=pid_config['horizontal']['kp'],
            ki=pid_config['horizontal']['ki'],
            kd=pid_config['horizontal']['kd'],
            setpoint=0.5,
            output_limits=(-300, 300)
        )
        self.pid_y: PID = PID(
            kp=pid_config['vertical']['kp'],
            ki=pid_config['vertical']['ki'],
            kd=pid_config['vertical']['kd'],
            setpoint=0.5,
            output_limits=(-300, 300)
        )

        # self.dead_zone: float = dead_zone
        self.horizontal_min_angle: int = camera_movement_config.get("horizontal_min_angle", 0)
        self.horizontal_max_angle: int = camera_movement_config.get("horizontal_max_angle", 270)
        self.vertical_min_angle: int = camera_movement_config.get("vertical_min_angle", 45)
        self.vertical_max_angle: int = camera_movement_config.get("vertical_max_angle", 135)
        
        self.fader_config = fader_config or {}
        # On peut stocker les sous-champs pour éviter de parser à chaque fois
        self.fader_enable = self.fader_config.get("enable", False)
        self.fader_max_distance = float(self.fader_config.get("max_distance", 0.5))
        self.fader_min_factor = float(self.fader_config.get("min_factor", 0.2))
        self.fader_max_factor = float(self.fader_config.get("max_factor", 1.0))


    def update_position(self, x_center: float, y_center: float) -> None:
        # 1) Calcul standard du PID
        x_correction: float = self.pid_x.update(x_center)
        y_correction: float = self.pid_y.update(y_center)

        # 2) Appliquer une limite de correction (existant)
        x_correction = max(-2.0, min(2.0, x_correction))
        y_correction = max(-2.0, min(2.0, y_correction))

        # 3) Fader (si enable == True)
        if self.fader_enable:
            # distance du centre
            dx = x_center - 0.5
            dy = y_center - 0.5
            distance = (dx*dx + dy*dy)**0.5  # sqrt(dx^2 + dy^2)
            
            gamma = 1.2  # Paramètre que vous pouvez mettre dans config.yaml si vous voulez
            
            # ratio dans [0,1]
            ratio = distance / self.fader_max_distance
            ratio = max(0.0, min(1.0, ratio))
            ratio_pow = ratio ** gamma
            
            # interpolation
            # ex. fader = min_factor + ratio * (max_factor - min_factor)
            # fader_value = self.fader_min_factor + ratio*(self.fader_max_factor - self.fader_min_factor)
            fader_value = self.fader_min_factor + ratio_pow * (self.fader_max_factor - self.fader_min_factor)

            # appliquer le fader
            x_correction *= fader_value
            y_correction *= fader_value

            logger.debug(
                f"[Fader] distance={distance:.3f} ratio={ratio:.3f} "
                f"fader={fader_value:.3f} => x_corr={x_correction:.3f}, y_corr={y_correction:.3f}"
            )

        # 4) Calculer l'angle final
        new_horizontal_angle = self.servo_horizontal.current_angle + x_correction
        new_horizontal_angle = max(
            self.horizontal_min_angle,
            min(self.horizontal_max_angle, new_horizontal_angle)
        )
        self.servo_horizontal.set_servo_angle(new_horizontal_angle)

        new_vertical_angle = self.servo_vertical.current_angle + y_correction
        new_vertical_angle = max(
            self.vertical_min_angle,
            min(self.vertical_max_angle, new_vertical_angle)
        )
        self.servo_vertical.set_servo_angle(new_vertical_angle)

        logger.debug(
            f"Camera Position Updated -> "
            f"X Correction: {x_correction:.4f}, "
            f"New Horizontal Angle: {new_horizontal_angle:.2f}°, "
            f"Y Correction: {y_correction:.4f}, "
            f"New Vertical Angle: {new_vertical_angle:.2f}°"
        )

    def position_zero(self) -> None:
        """
        Place la caméra à une position de référence.
        """
        self.servo_horizontal.set_servo_angle(self.horizontal_max_angle / 2)
        self.servo_vertical.set_servo_angle((self.vertical_min_angle + self.vertical_max_angle) / 2)
        time.sleep(1)
        # logger.info("Camera positioned to zero.")

    def position_turn(self) -> None:
        """
        Effectue un mouvement de balayage de la caméra.
        """
        self.servo_horizontal.set_servo_angle(self.horizontal_min_angle)
        self.servo_vertical.set_servo_angle(self.vertical_min_angle)
        time.sleep(1)
        self.servo_horizontal.set_servo_angle(self.horizontal_max_angle)
        self.servo_vertical.set_servo_angle(self.vertical_max_angle)
        time.sleep(1)
        self.position_zero()
        logger.info("Camera performed turn sweep.")

    def cleanup_servo(self) -> None:
        """
        Nettoie les ressources des servos et réinitialise les PID.
        """
        self.servo_horizontal.cleanup()
        self.servo_vertical.cleanup()
        self.pid_x.reset()
        self.pid_y.reset()
        logger.info("Camera deplacement cleanup completed.")

# ===================================
# ======= CAMERA CONTROLLER ========
# ===================================
class CameraController:
    """
    Pilote la caméra en fonction des coordonnées fournies.
    """
    def __init__(
        self, 
        camera_deplacement: CameraDeplacement, 
        user_data: UserAppCallback,
        segment_recorder
        ) -> None:
        self.camera_deplacement: CameraDeplacement = camera_deplacement
        self.current_center: Optional[Tuple[float, float]] = None
        self.time_since_update: Optional[int] = None
        self.lock: threading.Lock = threading.Lock()
        self.running: bool = True
        self.enable_movement: bool = CAMERA_MOVEMENT_ENABLE
        
        self.segment_recorder = segment_recorder  # instance de HlsSegmentRecorder
        self.detection_active = False
        self.no_detection_timer = None
        self.no_detection_delay = 7.0  # 10s
        self.detected_objects = set()
        
        self.valid_detection_frame_counts = defaultdict(int)  # Compteur par classe
        self.detection_active_classes = set()  # Classes actuellement enregistrement
        
        # Définir les seuils de frames par classe directement dans le code
        self.frame_thresholds = {
            "person": 20,
            "cat": 30  # Augmenté pour réduire les faux positifs
        }
        
        self.movement = (0, 0)  # Position par défaut
        # Attributs pour la gestion des logs significatifs
        self.last_logged_movement: Optional[Tuple[float, float]] = None
        self.movement_threshold: float = 0.03  # Seuil de mouvement significatif (ajustez selon vos besoins)
        
        self.persistent_mode: int = CAMERA_MODE  # mode demandé dans config
        self.camera_mode: int = CAMERA_MODE  # mode effectif au runtime
        self.last_detection_time: float = 0.0
        self.detection_timeout: float = 5.0  # 5s sans détection => retour mode 1 (si persistent_mode=1)

        # Variables pour le balayage (Mode 1)
        self.scan_speed = CAMERA_MOVEMENT.get("scan_speed", 2)   # degrés par itération
        self.scan_period = CAMERA_MOVEMENT.get("scan_period", 10)  # secondes entre balayages
        self.next_scan_time = 0.0
        self.scanning_in_progress = False
        self.scanning_angle = 0.0
        self.scanning_direction = +1  # +1 = va vers la droite, -1 = retour
        self.detection_count = 0
        self.required_persistent_frames = 2  # au choix : 2 ou 3 frames
        self.scanning_reversals = 0
        self.scanning_to_center = False
        self.next_scan_time = 0.0
        self.user_data: UserAppCallback = user_data
        self.thread: threading.Thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set_persistent_mode(self, mode: int) -> None:
        with self.lock:
            self.persistent_mode = mode
            logger.info(f"CameraController -> persistent_mode={mode}")

            if mode == 2:
                # on se cale tout de suite en mode 2
                self.camera_mode = 2
            elif mode == 0:
                # on se cale en mode 0
                self.camera_mode = 0
            else:
                # mode=1
                self.camera_mode = 1
                
    def set_mode(self, mode: int) -> None:
        with self.lock:
            self.camera_mode = mode
            logger.info(f"CameraController passe en mode={mode}")
            
    def set_enable_movement(self, enable: bool) -> None:
        with self.lock:
            self.enable_movement = enable

    def no_detection_timeout(self):
        """
        Appelé après X sec sans détection. On arrête donc l'enregistrement.
        """
        with self.user_data.lock:
            if self.detection_active:
                # Calculer le total des frames valides
                total_valid_frames = sum(self.valid_detection_frame_counts.values())
                self.segment_recorder.stop_detection_and_finalize(valid_frame_counts=self.valid_detection_frame_counts)
                self.detection_active = False
                self.detected_objects.clear()
                self.valid_detection_frame_counts.clear()
                logger.info("Timeout sans détection : enregistrement arrêté.")
        
        # Réinitialiser tous les compteurs de frames
        self.reset_valid_frame_count()


    def reset_valid_frame_count(self):
        """
        Réinitialise les compteurs de frames valides pour toutes les classes.
        """
        with self.lock:
            for label in self.valid_detection_frame_counts:
                self.valid_detection_frame_counts[label] = 0
            logger.debug("valid_detection_frame_counts réinitialisés via reset_valid_frame_count().")

                
    def update_info(self, center: Optional[Tuple[float, float]], time_since_update: Optional[int], label_list=None) -> None:
        with self.lock:
            self.current_center = center
            self.time_since_update = time_since_update
            logger.debug(f"CameraController updated info -> Center: {center}, Time Since Update: {time_since_update}")

        with self.user_data.lock:
            if center is not None and time_since_update == 0 and label_list:
                for label in label_list:
                    if label in TRACK_OBJECTS:
                        self.valid_detection_frame_counts[label] += 1
                        logger.debug(f"Frame valide pour '{label}': {self.valid_detection_frame_counts[label]}/{self.frame_thresholds.get(label, 30)}")
                        
                        # Vérifier si le seuil est atteint pour cette classe
                        if (self.valid_detection_frame_counts[label] >= self.frame_thresholds.get(label, 30) and
                            not self.detection_active):
                            self.detection_active = True
                            self.segment_recorder.start_detection()  # Démarrer la copie des segments
                            logger.info(f"Détection validée pour '{label}'. Enregistrement démarré.")
                        
                # Annuler le timer de non-détection
                if self.no_detection_timer:
                    self.no_detection_timer.cancel()
                    self.no_detection_timer = None

            else:
                # Plus de détection sur cette frame
                if self.detection_active:
                    if not self.no_detection_timer:
                        self.no_detection_timer = threading.Timer(
                            self.no_detection_delay,
                            self.no_detection_timeout
                        )
                        self.no_detection_timer.start()

    def run(self) -> None:
        while self.running:
            with self.lock:
                center = self.current_center
                time_since_update = self.time_since_update
                camera_mode = self.camera_mode
                dead_zone = self.user_data.dead_zone
                p_mode = self.persistent_mode  # persistent mode

            # ********************* LOGIQUE SUR LA DÉTECTION PERSISTANTE *********************
            if camera_mode == 1:
                # Si on a une détection sur cette frame
                if center is not None and time_since_update == 0:
                    self.detection_count += 1
                    if self.detection_count >= self.required_persistent_frames:
                        # On bascule en mode 2 (suivi), car on a rencontré
                        # une détection "persistante" plus de N frames
                        with self.lock:
                            self.camera_mode = 2
                        logger.info(
                            f"Détection persistante ({self.detection_count} frames). "
                            f"Passage en mode 2 (suivi)."
                        )
                else:
                    # Pas de détection ou plus mise à jour => on réinitialise
                    self.detection_count = 0
            
            # ********************* GESTION DES 3 MODES *********************
            if camera_mode == 0:
                self.camera_deplacement.position_zero()
                time.sleep(0.5)
                continue

            elif camera_mode == 1:
                # On n’entre ici que si on n’a pas forcé le passage en mode 2.
                # => procéder au balayage
                self.handle_scanning()

            # Mode 2 = suivi
            else:  # camera_mode == 2
                if center is not None and time_since_update == 0:
                    # Mise à jour last_detection_time
                    self.last_detection_time = time.time()

                    x_center, y_center = center
                    if (abs(x_center - 0.5) > dead_zone or abs(y_center - 0.5) > dead_zone):
                        self.camera_deplacement.update_position(x_center, y_center)
                    else:
                        self.camera_deplacement.update_position(0.5, 0.5)
                else:
                    # Aucune détection => se replacer au centre
                    self.camera_deplacement.update_position(0.5, 0.5)

                    # Vérifier s'il faut repasser en mode 1
                    # UNIQUEMENT si persistent_mode == 1
                    if p_mode == 1:
                        now = time.time()
                        if now - self.last_detection_time > self.detection_timeout:
                            with self.lock:
                                self.camera_mode = 1
                            logger.info("Aucune détection récente : retour au mode 1 (balayage).")

            time.sleep(0.01)
                
            # Gestion des logs significatifs
            if self.is_significant_movement(self.movement):
                logger.info(f"Déplacement de la caméra -> Centre: {self.movement}")
                self.last_logged_movement = self.movement
                
            time.sleep(0.01)  # Ajuster la fréquence selon les besoins

    def handle_scanning(self) -> None:
        """
        Gère un balayage horizontal qui démarre à l'angle actuel
        et fait 2 allers-retours, puis revient au centre.
        """
        now = time.time()

        # Calcul de l'angle central
        center_angle = (self.camera_deplacement.horizontal_min_angle +
                        self.camera_deplacement.horizontal_max_angle) / 2.0

        # 1) Si on n'est pas déjà en cours de balayage
        if not self.scanning_in_progress:
            # Vérifier si c'est l'heure de lancer un nouveau cycle de balayage
            if now < self.next_scan_time:
                # On attend la prochaine fenêtre
                return
            
            # => On démarre un nouveau balayage
            self.scanning_in_progress = True
            # On prend la position actuelle du servo horizontal
            self.scanning_angle = self.camera_deplacement.servo_horizontal.current_angle
            
            # Déterminer la direction initiale en fonction de la position vs le centre
            if self.scanning_angle < center_angle:
                self.scanning_direction = +1
            else:
                self.scanning_direction = -1
            
            self.scanning_reversals = 0
            self.scanning_to_center = False
            
            logger.info(
                f"Nouvelle séquence de balayage: angle_depart={self.scanning_angle:.1f}, "
                f"direction={self.scanning_direction}, time={now}"
            )
        
        # 2) Si on est au stade "retourner au centre" (après 2 allers-retours)
        if self.scanning_to_center:
            # On bouge doucement vers l'angle central
            desired = center_angle
            step = self.scan_speed
            # Signe du déplacement
            direction_to_center = +1 if (desired > self.scanning_angle) else -1

            new_angle = self.scanning_angle + direction_to_center * step
            # Si on dépasse la cible, on se cale pile sur le center_angle
            if (direction_to_center > 0 and new_angle >= desired) or \
               (direction_to_center < 0 and new_angle <= desired):
                # On a atteint le centre
                new_angle = desired
                self.scanning_in_progress = False
                self.scanning_to_center = False
                self.next_scan_time = now + self.scan_period
                logger.info(
                    f"Balayage terminé et positionné au centre (angle={new_angle:.1f}). "
                    f"Prochain balayage après {self.scan_period}s."
                )
            
            # Appliquer l'angle
            self.scanning_angle = new_angle
            self.camera_deplacement.servo_horizontal.set_servo_angle(self.scanning_angle)
            # Option: on garde le vertical "centré" aussi
            self.camera_deplacement.servo_vertical.set_servo_angle(
                (self.camera_deplacement.vertical_min_angle + 
                 self.camera_deplacement.vertical_max_angle) / 2.0
            )
            return
        
        # 3) Sinon, on est en plein balayage "gauche-droite" ou "droite-gauche"
        step = self.scan_speed
        new_angle = self.scanning_angle + self.scanning_direction * step
        
        # Vérifier limites
        if new_angle >= self.camera_deplacement.horizontal_max_angle:
            new_angle = self.camera_deplacement.horizontal_max_angle
            self.scanning_reversals += 1

            if self.scanning_reversals < 2:
                # On inverse la direction pour continuer
                self.scanning_direction = -1
            else:
                # On enclenche le retour au centre
                self.scanning_to_center = True
        
        elif new_angle <= self.camera_deplacement.horizontal_min_angle:
            new_angle = self.camera_deplacement.horizontal_min_angle
            self.scanning_reversals += 1

            if self.scanning_reversals < 2:
                self.scanning_direction = +1
            else:
                self.scanning_to_center = True

        # Appliquer
        self.scanning_angle = new_angle
        self.camera_deplacement.servo_horizontal.set_servo_angle(self.scanning_angle)
        self.camera_deplacement.servo_vertical.set_servo_angle(
            (self.camera_deplacement.vertical_min_angle + 
             self.camera_deplacement.vertical_max_angle) / 2.0
        )
        
    def is_significant_movement(self, current_movement: Tuple[float, float]) -> bool:
        """
        Détermine si le déplacement actuel est significatif par rapport au dernier déplacement loggué.
        """
        if self.last_logged_movement is None:
            return True  # Toujours loguer la première occurrence

        delta_x = abs(current_movement[0] - self.last_logged_movement[0])
        delta_y = abs(current_movement[1] - self.last_logged_movement[1])

        return delta_x >= self.movement_threshold or delta_y >= self.movement_threshold

    def stop(self) -> None:
        self.running = False
        if self.no_detection_timer:
            self.no_detection_timer.cancel()
        self.thread.join()
        self.segment_recorder.stop_all()
        logger.info("CameraController thread stopped.")

# =============================
# ======= CONFIGURATION ========
# =============================
def load_config(config_path: str = "config.yaml") -> dict:
    """
    Charge la configuration depuis un fichier YAML.

    Args:
        config_path (str): Nom du fichier de configuration.

    Returns:
        dict: Dictionnaire contenant les configurations.
    """
    try:
        # Obtenir le chemin absolu basé sur l'emplacement du script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        absolute_config_path = os.path.join(script_dir, config_path)

        # Ajoutez une ligne de diagnostic pour vérifier le chemin
        logger.debug(f"Chemin absolu du fichier de configuration: {absolute_config_path}")

        with open(absolute_config_path, 'r') as file:
            config = yaml.safe_load(file)
            # logger.info(f"Configuration chargée depuis {absolute_config_path}")
            return config
    except FileNotFoundError:
        logger.error(f"Fichier de configuration {absolute_config_path} non trouvé.")
        sys.exit(1)
    except yaml.YAMLError as exc:
        logger.error(f"Erreur lors du chargement du fichier YAML: {exc}")
        sys.exit(1)

# Chargement initial de la configuration
config = load_config("config.yaml")

# Variables de configuration
TRACK_OBJECTS: List[str] = config.get("track_objects", ["person", "cat"])
CONFIDENCE_THRESHOLD: float = config.get("confidence_threshold", 0.5)
DEAD_ZONE: float = config.get("dead_zone", 0.05)

# PID Parameters
PID_HORIZONTAL = config.get("pid", {}).get("horizontal", {})
PID_VERTICAL = config.get("pid", {}).get("vertical", {})
FADER_CONFIG = config.get("fader", {})

# Servo Parameters
SERVO_CONFIG = config.get("servo", {})

# Camera Movement Parameters
CAMERA_MOVEMENT = config.get("camera_movement", {})
CAMERA_MOVEMENT_ENABLE: bool = CAMERA_MOVEMENT.get("enable", True)
CAMERA_MODE: int = CAMERA_MOVEMENT.get("mode", 2)  # 2 = valeur par défaut (suivi)

# Window Mover Parameters
WINDOW_MOVER_CONFIG = config.get("window_mover", {})

# Tracking Parameters
TRACKING_CONFIG = config.get("tracking", {})
MAX_AGE: int = TRACKING_CONFIG.get("max_age", 30)
MIN_HITS: int = TRACKING_CONFIG.get("min_hits", 3)
IOU_THRESHOLD: float = TRACKING_CONFIG.get("iou_threshold", 0.3)
DT: float = TRACKING_CONFIG.get("dt", 1/25)
KALMAN_Q: list = TRACKING_CONFIG.get("kalman_filter", {}).get("Q", None)
KALMAN_R: list = TRACKING_CONFIG.get("kalman_filter", {}).get("R", None)

# FRAME_THRESHOLDS: dict = config.get("frame_thresholds", {"person": 20, "cat": 20})

# =============================
# ======= EVENT CONFIG FILE ======
# =============================
class ConfigHandler(FileSystemEventHandler):
    """
    Gestionnaire d'événements pour les modifications du fichier de configuration.
    """
    def __init__(self, config_path: str, reload_callback):
        super().__init__()
        self.config_path = os.path.abspath(config_path)  # Utiliser un chemin absolu
        self.reload_callback = reload_callback

    def on_modified(self, event):
        event_path = os.path.abspath(event.src_path)
        if event_path == self.config_path:
            self.reload_callback()

# =============================
# ======= RUN WATCHER =========
# =============================
def start_config_watcher(config_path: str, reload_callback) -> None:
    """
    Démarre un observateur pour surveiller les modifications du fichier de configuration.

    Args:
        config_path (str): Chemin vers le fichier de configuration.
        reload_callback (callable): Fonction à appeler pour recharger la configuration.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    absolute_config_path = os.path.join(script_dir, config_path)
    event_handler = ConfigHandler(absolute_config_path, reload_callback)
    observer = Observer()
    observer.schedule(event_handler, path=script_dir, recursive=False)  # Surveille le répertoire absolu
    observer.start()
    logger.info(f"Observateur de configuration démarré pour {absolute_config_path}.")

    # Garder l'observateur en marche dans un thread séparé
    def watch():
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()

    watch_thread = threading.Thread(target=watch, daemon=True)
    watch_thread.start()


def reload_config(detection_app: 'DetectionApp', config_path: str = "config.yaml") -> None:
    """
    Recharge la configuration et met à jour les paramètres de l'application.
    Si une valeur dans le fichier config.yaml est invalide, on garde l'ancienne valeur ou une valeur par défaut,
    et on loggue l'information. Si elle est valide, on l'applique et on loggue le changement.
    
    Args:
        detection_app (DetectionApp): Instance de l'application de détection.
        config_path (str): Chemin vers le fichier de configuration.
    """
    global config, TRACK_OBJECTS, CONFIDENCE_THRESHOLD, DEAD_ZONE, CAMERA_MOVEMENT_ENABLE, CAMERA_MODE

    # Sauvegarde des valeurs actuelles (pour log et fallback)
    old_track_objects = TRACK_OBJECTS
    old_conf_threshold = CONFIDENCE_THRESHOLD
    old_dead_zone = DEAD_ZONE
    old_cam_movement_enable = CAMERA_MOVEMENT_ENABLE
    old_cam_mode = CAMERA_MODE

    # Lecture (et parsing YAML) du nouveau config
    new_config = load_config(config_path)

    # Vérifier que c'est bien un dict
    if not isinstance(new_config, dict):
        logger.error("La configuration rechargée n'est pas un dictionnaire valide. (Aucun changement appliqué)")
        return

    # On met à jour la variable globale config
    config = new_config

    # ============================================
    #      1) Mise à jour des variables globales
    # ============================================

    # 1.1) TRACK_OBJECTS
    new_track_objects = new_config.get("track_objects", old_track_objects)
    if isinstance(new_track_objects, list) and all(isinstance(obj, str) for obj in new_track_objects):
        TRACK_OBJECTS = new_track_objects
        # Log si la valeur a changé
        if TRACK_OBJECTS != old_track_objects:
            logger.info(f"track_objects mis à jour : {old_track_objects} -> {TRACK_OBJECTS}")
    else:
        logger.warning(f"track_objects invalide, on conserve {old_track_objects}")

    # 1.2) CONFIDENCE_THRESHOLD
    new_conf_threshold = new_config.get("confidence_threshold", old_conf_threshold)
    if isinstance(new_conf_threshold, (int, float)):
        CONFIDENCE_THRESHOLD = float(new_conf_threshold)
        if CONFIDENCE_THRESHOLD != old_conf_threshold:
            logger.info(f"confidence_threshold mis à jour : {old_conf_threshold} -> {CONFIDENCE_THRESHOLD}")
    else:
        logger.warning(f"confidence_threshold invalide, on conserve {old_conf_threshold}")

    # 1.3) DEAD_ZONE
    new_dead_zone = new_config.get("dead_zone", old_dead_zone)
    if isinstance(new_dead_zone, (int, float)):
        # On peut forcer l'intervalle [0,1] si besoin
        if 0 <= new_dead_zone <= 1:
            DEAD_ZONE = new_dead_zone
            if DEAD_ZONE != old_dead_zone:
                logger.info(f"dead_zone mis à jour : {old_dead_zone} -> {DEAD_ZONE}")
        else:
            logger.warning(f"dead_zone hors intervalle [0,1], on conserve {old_dead_zone}")
    else:
        logger.warning(f"dead_zone invalide, on conserve {old_dead_zone}")

    # ============================================
    #   2) Mise à jour de CAMERA_MOVEMENT_ENABLE
    #      et CAMERA_MODE
    # ============================================
    camera_movement_section = new_config.get("camera_movement", {})
    if isinstance(camera_movement_section, dict):
        # 2.1) enable
        new_cam_movement_enable = camera_movement_section.get("enable", old_cam_movement_enable)
        new_scan_speed = camera_movement_section.get("scan_speed", None)
        new_scan_period = camera_movement_section.get("scan_period", None)
        if isinstance(new_cam_movement_enable, bool):
            CAMERA_MOVEMENT_ENABLE = new_cam_movement_enable
            if CAMERA_MOVEMENT_ENABLE != old_cam_movement_enable:
                logger.info(
                    f"CAMERA_MOVEMENT_ENABLE mis à jour : "
                    f"{old_cam_movement_enable} -> {CAMERA_MOVEMENT_ENABLE}"
                )
        else:
            logger.warning(
                f"camera_movement.enable est invalide, on conserve {old_cam_movement_enable}"
            )

        # 2.2) mode (0,1,2)
        new_cam_mode = camera_movement_section.get("mode", old_cam_mode)
        if isinstance(new_cam_mode, int) and new_cam_mode in [0, 1, 2]:
            CAMERA_MODE = new_cam_mode
            if CAMERA_MODE != old_cam_mode:
                logger.info(f"CAMERA_MODE mis à jour : {old_cam_mode} -> {CAMERA_MODE}")
        else:
            logger.warning(
                f"camera_movement.mode est invalide ou hors plage [0,1,2], "
                f"on conserve {old_cam_mode}"
            )
    else:
        logger.warning("camera_movement n'est pas un dict, on garde l'ancienne configuration.")

    # Log final sur l'état du mouvement
    logger.info(f"CameraController movement enabled: {CAMERA_MOVEMENT_ENABLE}, mode: {CAMERA_MODE}")

    # =========================================================
    #  3) Mise à jour de l'état CameraController (si présent)
    # =========================================================
    if detection_app.state.user_data.camera_controller:
        # On met d'abord à jour l'activation (héritée de l'ancien code)
        detection_app.state.user_data.camera_controller.set_enable_movement(CAMERA_MOVEMENT_ENABLE)
        # Puis on met à jour le mode
        detection_app.state.user_data.camera_controller.set_persistent_mode(CAMERA_MODE)
        camera_controller = detection_app.state.user_data.camera_controller
        
        # enable mouvement + mode
        camera_controller.set_enable_movement(CAMERA_MOVEMENT_ENABLE)
        camera_controller.set_persistent_mode(CAMERA_MODE)
            # Appliquer scan_speed s’il est valide
        if isinstance(new_scan_speed, (int, float)):
            camera_controller.scan_speed = float(new_scan_speed)
            logger.info(f"scan_speed mis à jour -> {camera_controller.scan_speed}")
        else:
            logger.info(f"scan_speed inchangé (valeur invalide ou non spécifiée).")
        
        # Appliquer scan_period s’il est valide
        if isinstance(new_scan_period, (int, float)):
            camera_controller.scan_period = float(new_scan_period)
            logger.info(f"scan_period mis à jour -> {camera_controller.scan_period}")
        else:
            logger.info(f"scan_period inchangé (valeur invalide ou non spécifiée).")

    # ============================================
    #  4) PID & camera_deplacement
    # ============================================
    camera_deplacement = detection_app.state.user_data.camera_controller.camera_deplacement

    # Mettre à jour le champ pid_config
    pid_config_candidate = new_config.get("pid", {})
    if isinstance(pid_config_candidate, dict):
        camera_deplacement.pid_config = pid_config_candidate
    else:
        logger.warning(
            "pid_config invalide, on conserve la précédente "
            "(aucun impact direct si on recrée les PID)."
        )

    # On récupère ce qu'on va mettre dans le nouveau PID
    new_pid_config = new_config.get("pid", {})
    horizontal_pid = new_pid_config.get("horizontal", {})
    vertical_pid = new_pid_config.get("vertical", {})

    # Sécuriser l'accès aux 3 coefficients horizontal
    kp_h = horizontal_pid.get("kp", 0)
    ki_h = horizontal_pid.get("ki", 0)
    kd_h = horizontal_pid.get("kd", 0)

    # Pareil vertical
    kp_v = vertical_pid.get("kp", 0)
    ki_v = vertical_pid.get("ki", 0)
    kd_v = vertical_pid.get("kd", 0)

    # On recrée les PID comme dans le code existant
    old_pid_x = (camera_deplacement.pid_x.kp,
                 camera_deplacement.pid_x.ki,
                 camera_deplacement.pid_x.kd)
    old_pid_y = (camera_deplacement.pid_y.kp,
                 camera_deplacement.pid_y.ki,
                 camera_deplacement.pid_y.kd)

    try:
        camera_deplacement.pid_x = PID(
            kp=float(kp_h),
            ki=float(ki_h),
            kd=float(kd_h),
            setpoint=0.5,
            output_limits=(-150, 150)
        )
        camera_deplacement.pid_y = PID(
            kp=float(kp_v),
            ki=float(ki_v),
            kd=float(kd_v),
            setpoint=0.5,
            output_limits=(-50, 50)
        )
        new_pid_x = (camera_deplacement.pid_x.kp,
                     camera_deplacement.pid_x.ki,
                     camera_deplacement.pid_x.kd)
        new_pid_y = (camera_deplacement.pid_y.kp,
                     camera_deplacement.pid_y.ki,
                     camera_deplacement.pid_y.kd)

        # Log si ça a changé
        if new_pid_x != old_pid_x:
            logger.info(f"PID X mis à jour : {old_pid_x} -> {new_pid_x}")
        if new_pid_y != old_pid_y:
            logger.info(f"PID Y mis à jour : {old_pid_y} -> {new_pid_y}")
    except Exception as e:
        logger.warning(
            f"Impossible de recréer les PID : {e}. On conserve les PID précédents."
        )

    # Mise à jour du fader
    fader_config = new_config.get("fader", {})
    camera_deplacement.fader_config = fader_config
    camera_deplacement.fader_enable = fader_config.get("enable", False)
    camera_deplacement.fader_max_distance = float(fader_config.get("max_distance", 0.5))
    camera_deplacement.fader_min_factor = float(fader_config.get("min_factor", 0.2))
    camera_deplacement.fader_max_factor = float(fader_config.get("max_factor", 1.0))

    # Zone morte dans user_data
    old_dead_zone_app = detection_app.state.user_data.dead_zone
    detection_app.state.user_data.dead_zone = DEAD_ZONE
    if detection_app.state.user_data.dead_zone != old_dead_zone_app:
        logger.info(
            f"dead_zone application mis à jour : "
            f"{old_dead_zone_app} -> {detection_app.state.user_data.dead_zone}"
        )

    # ============================================
    #  5) Angles min/max de la caméra
    # ============================================
    camera_deplacement.horizontal_min_angle = camera_movement_section.get("horizontal_min_angle", 0)
    camera_deplacement.horizontal_max_angle = camera_movement_section.get("horizontal_max_angle", 270)
    camera_deplacement.vertical_min_angle = camera_movement_section.get("vertical_min_angle", 45)
    camera_deplacement.vertical_max_angle = camera_movement_section.get("vertical_max_angle", 135)

    # ============================================
    #  6) Servos
    # ============================================
    servo_config_candidate = new_config.get("servo", {})
    if isinstance(servo_config_candidate, dict):
        old_chan_h = camera_deplacement.servo_horizontal.channel
        old_chan_v = camera_deplacement.servo_vertical.channel

        new_chan_h = servo_config_candidate.get("channel_horizontal", old_chan_h)
        new_chan_v = servo_config_candidate.get("channel_vertical", old_chan_v)

        camera_deplacement.servo_horizontal.channel = new_chan_h
        camera_deplacement.servo_vertical.channel = new_chan_v

        # Log si ça a changé
        if new_chan_h != old_chan_h:
            logger.info(f"servo_horizontal.channel mis à jour : {old_chan_h} -> {new_chan_h}")
        if new_chan_v != old_chan_v:
            logger.info(f"servo_vertical.channel mis à jour : {old_chan_v} -> {new_chan_v}")

        # On repositionne les angles
        camera_deplacement.servo_horizontal.set_servo_angle(
            camera_deplacement.servo_horizontal.current_angle
        )
        camera_deplacement.servo_vertical.set_servo_angle(
            camera_deplacement.servo_vertical.current_angle
        )
    else:
        logger.warning("servo n'est pas un dict valide, on garde les canaux servo existants.")

    # ============================================
    #  7) Paramètres de suivi (tracking)
    # ============================================
    tracking_config = new_config.get("tracking", {})
    if not isinstance(tracking_config, dict):
        logger.warning(
            "tracking n'est pas un dictionnaire valide, "
            "on conserve les anciens paramètres tracker."
        )
        tracking_config = {}

    max_age = tracking_config.get("max_age", 30)
    min_hits = tracking_config.get("min_hits", 3)
    iou_threshold = tracking_config.get("iou_threshold", 0.3)
    dt = tracking_config.get("dt", 1/25)
    kalman_section = tracking_config.get("kalman_filter", {})
    if not isinstance(kalman_section, dict):
        kalman_section = {}
    Q = kalman_section.get("Q", None)
    R = kalman_section.get("R", None)

    # Mettre à jour le tracker DeepSORT
    if detection_app.state.user_data.deep_sort_tracker:
        detection_app.state.user_data.deep_sort_tracker.update_tracker_params(
            max_age=max_age,
            min_hits=min_hits,
            iou_threshold=iou_threshold,
            dt=dt,
            Q=Q,
            R=R
        )

    logger.info("Configuration rechargée et appliquée (avec vérification et fallback).")

# ========================================
# =============== MAIN ===================
# ========================================
class DetectionApp:
    """
    Classe principale pour gérer l'application de détection.
    """

    def __init__(self) -> None:
        self.state: DetectionAppState = DetectionAppState()
        self.state.user_data = UserAppCallback()
        self.state.user_data.detection_app = self  # Lier l'instance DetectionApp
        self.state.user_data.deep_sort_tracker = DeepSORTTracker(
            max_age=MAX_AGE,
            min_hits=MIN_HITS,
            iou_threshold=IOU_THRESHOLD,
            dt=DT,
            Q=KALMAN_Q,
            R=KALMAN_R
        )
        self.app: GStreamerDetectionApp = GStreamerDetectionApp(app_callback, self.state.user_data)
        logger.info("DetectionApp initialisée.")

    def signal_handler(self, signum: int, frame: Any) -> None:
        logger.info(f"Signal {signum} reçu. Fermeture de l'application...")
        self.state.should_exit = True
        self.quit_app()

    def quit_app(self) -> None:
        if self.state.should_exit:
            logger.info("Arrêt de l'application...")
            if self.state.user_data.camera_controller:
                self.state.user_data.camera_controller.stop()
                self.state.user_data.camera_controller.camera_deplacement.position_zero()
                time.sleep(0.5)
                self.state.user_data.camera_controller.camera_deplacement.cleanup_servo()
            self.app.shutdown()  # Arrêter le GLib.MainLoop

    def move_window(self) -> None:
        """
        Déplace la fenêtre spécifiée dans la configuration aux coordonnées définies.
        """
        window_name = WINDOW_MOVER_CONFIG.get("window_name", "Hailo Detection App")
        move_x = WINDOW_MOVER_CONFIG.get("move_x", 440)
        move_y = WINDOW_MOVER_CONFIG.get("move_y", 62)
        max_retries = WINDOW_MOVER_CONFIG.get("max_retries", 10)
        delay = WINDOW_MOVER_CONFIG.get("delay", 3)

        attempts = 0
        while attempts < max_retries and not self.state.should_exit:
            try:
                window_ids = subprocess.check_output(
                    ['xdotool', 'search', '--name', window_name]
                ).decode().strip().split('\n')
                if window_ids and window_ids[0]:
                    window_id = window_ids[0]
                    subprocess.run(['xdotool', 'windowmove', window_id, str(move_x), str(move_y)])
                    logger.info(f"Fenêtre déplacée : ID {window_id} vers ({move_x}, {move_y})")
                    return
                else:
                    logger.warning(f"Fenêtre '{window_name}' non trouvée, tentative suivante...")
            except subprocess.CalledProcessError:
                logger.error(f"Erreur lors de la recherche de la fenêtre '{window_name}', tentative suivante...")
            attempts += 1
            time.sleep(delay)
        logger.error(f"Échec de déplacer la fenêtre '{window_name}' après plusieurs tentatives.")

    def move_window_thread(self) -> None:
        """
        Thread pour déplacer la fenêtre vidéo.
        """
        self.move_window()

    def update_configuration(self, new_config: dict) -> None:
        """
        Met à jour les configurations de l'application avec les nouvelles valeurs.

        Args:
            new_config (dict): Nouveau dictionnaire de configurations.
        """
        global TRACK_OBJECTS, CONFIDENCE_THRESHOLD, DEAD_ZONE, CAMERA_MOVEMENT_ENABLE
        TRACK_OBJECTS = new_config.get("track_objects", ["person", "cat"])
        CONFIDENCE_THRESHOLD = new_config.get("confidence_threshold", 0.5)
        DEAD_ZONE = new_config.get("dead_zone", 0.05)
        SIZE_THRESHOLD = new_config.get("size_threshold", 100)

        # Mettre à jour l'activation des mouvements de la caméra
        CAMERA_MOVEMENT = new_config.get("camera_movement", {})
        CAMERA_MOVEMENT_ENABLE = CAMERA_MOVEMENT.get("enable", True)

        # Mettre à jour l'état du CameraController
        if self.state.user_data.camera_controller:
            self.state.user_data.camera_controller.set_enable_movement(CAMERA_MOVEMENT_ENABLE)

        # Mettre à jour les PID configurations
        camera_deplacement = self.state.user_data.camera_controller.camera_deplacement
        camera_deplacement.pid_config = new_config.get("pid", {})

        # Mettre à jour les seuils de zone morte
        self.state.user_data.dead_zone = new_config.get("dead_zone", 0.05)

        # Mettre à jour les angles min/max
        camera_deplacement.horizontal_min_angle = new_config.get("camera_movement", {}).get("horizontal_min_angle", 0)
        camera_deplacement.horizontal_max_angle = new_config.get("camera_movement", {}).get("horizontal_max_angle", 270)
        camera_deplacement.vertical_min_angle = new_config.get("camera_movement", {}).get("vertical_min_angle", 45)
        camera_deplacement.vertical_max_angle = new_config.get("camera_movement", {}).get("vertical_max_angle", 135)

        # Mettre à jour les servos avec les nouvelles configurations
        camera_deplacement.servo_horizontal.channel = new_config.get("servo", {}).get("channel_horizontal", 0)
        camera_deplacement.servo_vertical.channel = new_config.get("servo", {}).get("channel_vertical", 1)
        camera_deplacement.servo_horizontal.set_servo_angle(camera_deplacement.servo_horizontal.current_angle)
        camera_deplacement.servo_vertical.set_servo_angle(camera_deplacement.servo_vertical.current_angle)
        
        # Mettre à jour les paramètres de suivi
        tracking_config = new_config.get("tracking", {})
        max_age = tracking_config.get("max_age", 30)
        min_hits = tracking_config.get("min_hits", 3)
        iou_threshold = tracking_config.get("iou_threshold", 0.3)
        dt = tracking_config.get("dt", 1/25)
        Q = tracking_config.get("kalman_filter", {}).get("Q", None)
        R = tracking_config.get("kalman_filter", {}).get("R", None)

        # Mettre à jour le tracker DeepSORT
        if detection_app.state.user_data.deep_sort_tracker:
            detection_app.state.user_data.deep_sort_tracker.update_tracker_params(
                max_age=max_age,
                min_hits=min_hits,
                iou_threshold=iou_threshold,
                dt=dt,
                Q=Q,
                R=R
            )

        logger.info("Configuration mise à jour dynamiquement.")

    def run(self) -> None:
        # Enregistrer les gestionnaires de signaux
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        logger.info("Gestionnaires de signaux enregistrés.")
        recording_frame_threshold = 20
        # Thread pour déplacer la fenêtre vidéo
        window_mover_thread = threading.Thread(target=self.move_window_thread, daemon=True)
        window_mover_thread.start()
        logger.info("Thread de déplacement de la fenêtre démarré.")

        # Initialiser le déplacement de la caméra
        camera_deplacement = CameraDeplacement(
            pid_config=config.get("pid", {}),
            servo_config=config.get("servo", {}),
            camera_movement_config=config.get("camera_movement", {}),
            fader_config=config.get("fader", {})
        )
        camera_deplacement.position_zero()
        # camera_deplacement.position_turn()
        logger.info("Déplacement de la caméra initialisé et positionné à zéro.")
        
        hls_recorder = HlsSegmentRecorder(
            hls_folder="/var/www/html/hailo_app/stream/hls",
            tmp_folder="/tmp/hls_segments",
            final_folder="/home/raspi/Videos/records",
            segment_extension=".ts",
            poll_interval=1.0
        )
        # Initialiser le TrackSelector et le CameraController
        self.state.user_data.track_selector = TrackSelector()
        self.state.user_data.camera_controller = CameraController(camera_deplacement, self.state.user_data, segment_recorder=hls_recorder)
        logger.info("TrackSelector et CameraController initialisés.")

        # Récupérer le cairo_overlay pour dessiner
        cairo_overlay = self.app.pipeline.get_by_name("cairo_overlay")
        if cairo_overlay is None:
            logger.error("Erreur : cairo_overlay non trouvé dans le pipeline.")
            sys.exit(1)
        cairo_overlay.connect("draw", draw_overlay, self.state.user_data)
        logger.info("Cairo overlay connecté.")

        # Démarrer le watcher de configuration
        start_config_watcher("config.yaml", lambda: reload_config(self, "config.yaml"))

        # Lancement de l'application GStreamer
        try:
            self.app.run()  # Démarrer le GLib.MainLoop
        except Exception as e:
            logger.exception(f"Exception rencontrée : {e}")
        finally:
            logger.info("Application fermée proprement.")
            self.quit_app()
            
    def handle_tracking_update(self, track):
        """
        Méthode supprimée pour centraliser le contrôle de la caméra via CameraController.
        """
        pass  # Cette méthode est désormais obsolète

# ========================================
# ======= APPLICATION CALLBACK ===========
# ========================================
def app_callback(pad: Gst.Pad, info: Gst.PadProbeInfo, user_data: UserAppCallback) -> Gst.PadProbeReturn:
    """
    Callback de l'application lorsqu'une nouvelle frame est disponible.
    """
    try:
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK

        user_data.increment()

        # Récupération des dimensions
        format, width, height = get_caps_from_pad(pad)
        if width and height:
            user_data.width = width
            user_data.height = height

        # Récupérer les détections
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
        user_data.last_detections = detections
        detection_labels = []
        for d in detections:
            if d.get_label() in TRACK_OBJECTS and d.get_confidence() >= CONFIDENCE_THRESHOLD:
                detection_labels.append(d.get_label())
                
        # Mise à jour des centres des bbox
        calculate_bbox_centers(user_data, CONFIDENCE_THRESHOLD)

        # Préparation des détections pour DeepSORT
        detection_bboxes = [
            [
                d.get_bbox().xmin(),
                d.get_bbox().ymin(),
                d.get_bbox().xmax(),
                d.get_bbox().ymax()
            ]
            for d in detections
            if d.get_label() in TRACK_OBJECTS and d.get_confidence() >= CONFIDENCE_THRESHOLD
        ]

        # Mise à jour du tracker
        user_data.tracker.update(detection_bboxes)

        # Récupération des pistes actives
        active_tracks = user_data.tracker.get_tracks()

        # Mise à jour du sélecteur de piste
        user_data.track_selector.update_tracks(active_tracks)

        # Obtenir les informations de la piste sélectionnée
        selected_center, time_since_update = user_data.track_selector.get_selected_track_info()
        if selected_center:
            user_data.camera_controller.update_info(
                selected_center,
                time_since_update,
                label_list=detection_labels)
        else:
            user_data.camera_controller.update_info(
                None, 
                None,
                label_list=detection_labels
            )

        return Gst.PadProbeReturn.OK
    except Exception as e:
        logger.exception(f"Erreur dans app_callback: {e}")
        return Gst.PadProbeReturn.OK  # Continue pipeline même en cas d'erreur

def calculate_bbox_centers(user_data: UserAppCallback, confidence_threshold: float = CONFIDENCE_THRESHOLD) -> None:
    """
    Calcule et stocke les centres normalisés des bounding boxes valides dans user_data.
    """
    user_data.bbox_centers = []

    detections = user_data.last_detections
    if not detections:
        return

    width = user_data.width
    height = user_data.height
    if not width or not height:
        return

    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        if label not in TRACK_OBJECTS or confidence < confidence_threshold:
            continue

        bbox = detection.get_bbox()
        center_x = 0.5 * (bbox.xmin() + bbox.xmax())
        center_y = 0.5 * (bbox.ymin() + bbox.ymax())

        # Normalisation des centres
        cx_norm = round(center_x, 4)  # Normaliser entre 0 et 1
        cy_norm = round(center_y, 4)
        user_data.bbox_centers.append((cx_norm, cy_norm))

    logger.debug(f"Calculated bbox centers: {user_data.bbox_centers}")

# ====================================================
# ======= AFFICHAGE DES POINTS DE CENTRE ET IDs ======
# ====================================================
def draw_overlay(cairo_overlay: Any, cr: cairo.Context, timestamp: int, duration: int, user_data: UserAppCallback) -> None:
    """
    Affiche la zone morte et les centres des bounding boxes ainsi que les pistes suivies.
    """
    if user_data.width is None or user_data.height is None:
        return

    width: int = user_data.width
    height: int = user_data.height
    # Dessin de la zone morte
    min_dimension: int = min(width, height)
    radius: float = user_data.dead_zone * min_dimension
    cr.set_source_rgb(*COLOR_BLUE)
    cr.arc(width / 2, height / 2, radius, 0, 2 * 3.14159)
    cr.stroke()

    # Affichage des barycentres bruts
    cr.set_source_rgb(*COLOR_RED)
    for center_x, center_y in user_data.bbox_centers:
        bx = center_x * width
        by = center_y * height
        cr.arc(bx, by, 5, 0, 2 * 3.14159)
        cr.fill()

    # --- AFFICHAGE FADER EN VIOLET ---
    # Suppose qu'on récupère fader_config dans user_data, ou alors 
    # on lit directement config["fader"]["max_distance"] si accessible.
    # Ex. (à adapter selon comment vous stockez le fader):
    fader_config = config.get("fader", {})
    fader_enable = fader_config.get("enable", False)
    fader_max_distance = fader_config.get("max_distance", 0.5)

    if fader_enable:
        # rayon pour le fader
        radius_fader = fader_max_distance * min_dimension

        # couleur violette (R, G, B)
        # par ex. violet = (1, 0, 1)
        cr.set_source_rgb(1, 0, 1)  
        
        cr.arc(width / 2, height / 2, radius_fader, 0, 2 * 3.14159)
        cr.stroke()
        
    cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(14)

    # Dessin des pistes suivies
    for track in user_data.tracker.get_tracks():
        center_x, center_y = track.center
        bx_f = center_x * width
        by_f = center_y * height
        color = COLOR_GREEN if track.time_since_update == 0 else COLOR_YELLOW
        cr.set_source_rgb(*color)
        cr.arc(bx_f, by_f, 4, 0, 2 * 3.14159)
        cr.fill()

        # Dessiner l'ID de la piste
        track_id = str(track.track_id)
        cr.move_to(bx_f + 10, by_f + 10)
        cr.show_text(track_id)

    logger.debug("Overlay drawn successfully.")

# ========================================
# =============== MAIN ===================
# ========================================
if __name__ == "__main__":
    detection_app = DetectionApp()
    detection_app.run()

# Fin de detection.py
