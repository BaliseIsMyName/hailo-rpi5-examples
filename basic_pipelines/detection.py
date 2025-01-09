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

import logging
from dataclasses import dataclass, field

# Configuration du logger
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

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
        output_limits: Tuple[float, float] = (-999, 999)
    ) -> None:
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.setpoint: float = setpoint
        self.output_limits: Tuple[float, float] = output_limits
        self._integral: float = 0.0
        self._last_error: float = 0.0
        self._last_time: float = time.time()

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

        logger.debug(f"PID Update -> Error: {error}, P: {p_out}, I: {i_out}, D: {d_out}, Output: {output}")
        return output

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

        self.channel: int = channel
        self.servo_min_us: int = servo_min_us
        self.servo_max_us: int = servo_max_us
        self.max_angle: int = max_angle

        self.current_angle: float = max_angle / 2.0
        self.set_servo_angle(self.current_angle)

    def _us_to_duty_cycle(self, pulse_us: int) -> int:
        period_us: int = 1_000_000 // self.pca.frequency  # ex: 20_000µs @ 50Hz
        duty_cycle: int = int((pulse_us / period_us) * 65535)
        return max(0, min(65535, duty_cycle))

    def set_servo_angle(self, angle_deg: float) -> None:
        angle_clamped: float = max(0, min(self.max_angle, angle_deg))
        self.current_angle = angle_clamped

        span_us: int = self.servo_max_us - self.servo_min_us
        pulse_us: float = self.servo_min_us + (span_us * (angle_clamped / float(self.max_angle)))
        self.pca.channels[self.channel].duty_cycle = self._us_to_duty_cycle(int(pulse_us))

        logger.debug(f"Servo Channel {self.channel} Angle set to {self.current_angle}° (Pulse: {pulse_us}µs)")

    def cleanup(self) -> None:
        # self.pca.channels[self.channel].duty_cycle = 0
        self.pca.deinit()
        logger.info(f"Servo Channel {self.channel} cleanup completed.")

# ===================================
# ======= CAMERA DEPLACEMENT ========
# ===================================
class CameraDeplacement:
    """
    Gère deux servos (horizontal + vertical) avec 2 PID.
    """

    def __init__(
        self,
        p_horizontal: float = 1.0,
        i_horizontal: float = 0.0,
        d_horizontal: float = 0.0,
        p_vertical: float = 1.0,
        i_vertical: float = 0.0,
        d_vertical: float = 0.0,
        dead_zone: float = 0.05,
        vertical_min_angle: int = 45,
        vertical_max_angle: int = 135,
        horizontal_min_angle: int = 0,
        horizontal_max_angle: int = 270
    ) -> None:
        self.servo_horizontal: ServoController = ServoController(
            channel=config.get("servo", {}).get("channel_horizontal", 0),
            max_angle=config.get("servo", {}).get("max_angle_horizontal", 270),
            freq=config.get("servo", {}).get("freq", 50),
            i2c_address=config.get("servo", {}).get("i2c_address", 0x40),
            servo_min_us=config.get("servo", {}).get("servo_min_us", 500),
            servo_max_us=config.get("servo", {}).get("servo_max_us", 2500)
        )
        self.servo_vertical: ServoController = ServoController(
            channel=config.get("servo", {}).get("channel_vertical", 1),
            max_angle=config.get("servo", {}).get("max_angle_vertical", 180),
            freq=config.get("servo", {}).get("freq", 50),
            i2c_address=config.get("servo", {}).get("i2c_address", 0x40),
            servo_min_us=config.get("servo", {}).get("servo_min_us", 500),
            servo_max_us=config.get("servo", {}).get("servo_max_us", 2500)
        )

        self.pid_x: PID = PID(
            kp=config.get("pid", {}).get("horizontal", {}).get("kp", 1.0),
            ki=config.get("pid", {}).get("horizontal", {}).get("ki", 0.0),
            kd=config.get("pid", {}).get("horizontal", {}).get("kd", 0.0),
            setpoint=0.5,
            output_limits=(-150, 150)
        )
        self.pid_y: PID = PID(
            kp=config.get("pid", {}).get("vertical", {}).get("kp", 1.0),
            ki=config.get("pid", {}).get("vertical", {}).get("ki", 0.0),
            kd=config.get("pid", {}).get("vertical", {}).get("kd", 0.0),
            setpoint=0.5,
            output_limits=(-50, 50)
        )

        self.dead_zone: float = dead_zone
        self.horizontal_min_angle: int = config.get("camera_movement", {}).get("horizontal_min_angle", 0)
        self.horizontal_max_angle: int = config.get("camera_movement", {}).get("horizontal_max_angle", 270)
        self.vertical_min_angle: int = config.get("camera_movement", {}).get("vertical_min_angle", 45)
        self.vertical_max_angle: int = config.get("camera_movement", {}).get("vertical_max_angle", 135)

    def update_position(self, x_center: float, y_center: float) -> None:
        """
        Mise à jour des servos en fonction de x_center et y_center (normalisés entre 0 et 1).
        """
        # Correction pour l'axe X
        x_correction: float = self.pid_x.update(x_center)
        new_horizontal_angle: float = self.servo_horizontal.current_angle + x_correction
        new_horizontal_angle = max(
            self.horizontal_min_angle,
            min(self.horizontal_max_angle, new_horizontal_angle)
        )
        self.servo_horizontal.set_servo_angle(new_horizontal_angle)

        # Correction pour l'axe Y
        y_correction: float = self.pid_y.update(y_center)
        new_vertical_angle: float = self.servo_vertical.current_angle + y_correction
        new_vertical_angle = max(
            self.vertical_min_angle,
            min(self.vertical_max_angle, new_vertical_angle)
        )
        self.servo_vertical.set_servo_angle(new_vertical_angle)

        logger.debug(f"Camera Position Updated -> Horizontal: {new_horizontal_angle}°, Vertical: {new_vertical_angle}°")

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

    def __init__(self, camera_deplacement: CameraDeplacement) -> None:
        self.camera_deplacement: CameraDeplacement = camera_deplacement
        self.current_center: Optional[Tuple[float, float]] = None
        self.time_since_update: Optional[int] = None
        self.lock: threading.Lock = threading.Lock()
        self.running: bool = True
        self.enable_movement: bool = CAMERA_MOVEMENT_ENABLE
        self.thread: threading.Thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        self.movement = (0, 0)  # Position par défaut
        # Attributs pour la gestion des logs significatifs
        self.last_logged_movement: Optional[Tuple[float, float]] = None
        self.movement_threshold: float = 0.03  # Seuil de mouvement significatif (ajustez selon vos besoins)
        
        # logger.info("CameraController thread started.")

    def set_enable_movement(self, enable: bool) -> None:
        with self.lock:
            self.enable_movement = enable
            # logger.info(f"CameraController movement enabled: {enable}")
            
    def update_info(self, center: Optional[Tuple[float, float]], time_since_update: Optional[int]) -> None:
        with self.lock:
            self.current_center = center
            self.time_since_update = time_since_update
            logger.debug(f"CameraController updated info -> Center: {center}, Time Since Update: {time_since_update}")

    def run(self) -> None:
        while self.running:
            with self.lock:
                center = self.current_center
                time_since_update = self.time_since_update
                enable_movement = self.enable_movement  # Récupérer l'état d'activation
            
            if enable_movement:
                if center is not None and time_since_update == 0:
                    x_center, y_center = center
                    # Vérifier si l'objet est en dehors de la zone morte
                    if (abs(x_center - 0.5) > self.camera_deplacement.dead_zone or
                            abs(y_center - 0.5) > self.camera_deplacement.dead_zone):
                        self.camera_deplacement.update_position(x_center, y_center)
                        self.movement = (x_center, y_center)
                    else:
                        # Stabiliser les servomoteurs
                        self.camera_deplacement.update_position(0.5, 0.5)
                else:
                    # Stabiliser les servomoteurs
                    self.camera_deplacement.update_position(0.5, 0.5)
            else:
                # Si les mouvements sont désactivés, maintenir la caméra en position zéro
                self.camera_deplacement.update_position(0.5, 0.5)
                
            # Gestion des logs significatifs
            if self.is_significant_movement(self.movement):
                logger.info(f"Déplacement de la caméra -> Centre: {self.movement}")
                self.last_logged_movement = self.movement
                
            time.sleep(0.1)  # Ajuster la fréquence selon les besoins

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
        self.thread.join()
        logger.info("CameraController thread stopped.")

# ========================================
# =========== USER APP CALLBACK ==========
# ========================================
class UserAppCallback(app_callback_class):
    """
    Classe de rappel utilisateur pour gérer les détections et le suivi.
    """

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

# Servo Parameters
SERVO_CONFIG = config.get("servo", {})

# Camera Movement Parameters
CAMERA_MOVEMENT = config.get("camera_movement", {})
CAMERA_MOVEMENT_ENABLE: bool = CAMERA_MOVEMENT.get("enable", True)


# Window Mover Parameters
WINDOW_MOVER_CONFIG = config.get("window_mover", {})

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

    Args:
        detection_app (DetectionApp): Instance de l'application de détection.
        config_path (str): Chemin vers le fichier de configuration.
    """
    global config, TRACK_OBJECTS, CONFIDENCE_THRESHOLD, DEAD_ZONE, CAMERA_MOVEMENT_ENABLE
    config = load_config(config_path)
    
    # Vérification que config est un dictionnaire
    if not isinstance(config, dict):
        logger.error("La configuration chargée n'est pas un dictionnaire valide.")
        return
    
    # Mettre à jour les variables globales
    TRACK_OBJECTS = config.get("track_objects", ["person", "cat"])
    CONFIDENCE_THRESHOLD = config.get("confidence_threshold", 0.5)
    DEAD_ZONE = config.get("dead_zone", 0.05)

    # Mettre à jour l'activation des mouvements de la caméra
    CAMERA_MOVEMENT = config.get("camera_movement", {})
    CAMERA_MOVEMENT_ENABLE = CAMERA_MOVEMENT.get("enable", True)

    # Mettre à jour l'état du CameraController
    if detection_app.state.user_data.camera_controller:
        detection_app.state.user_data.camera_controller.set_enable_movement(CAMERA_MOVEMENT_ENABLE)

    # Mettre à jour les PID
    detection_app.state.user_data.camera_controller.camera_deplacement.pid_x.kp = config.get("pid", {}).get("horizontal", {}).get("kp", 1.0)
    detection_app.state.user_data.camera_controller.camera_deplacement.pid_x.ki = config.get("pid", {}).get("horizontal", {}).get("ki", 0.0)
    detection_app.state.user_data.camera_controller.camera_deplacement.pid_x.kd = config.get("pid", {}).get("horizontal", {}).get("kd", 0.0)

    detection_app.state.user_data.camera_controller.camera_deplacement.pid_y.kp = config.get("pid", {}).get("vertical", {}).get("kp", 1.0)
    detection_app.state.user_data.camera_controller.camera_deplacement.pid_y.ki = config.get("pid", {}).get("vertical", {}).get("ki", 0.0)
    detection_app.state.user_data.camera_controller.camera_deplacement.pid_y.kd = config.get("pid", {}).get("vertical", {}).get("kd", 0.0)

    # Mettre à jour les seuils de zone morte
    detection_app.state.user_data.dead_zone = config.get("dead_zone", 0.05)

    # Mettre à jour les angles min/max
    detection_app.state.user_data.camera_controller.camera_deplacement.horizontal_min_angle = config.get("camera_movement", {}).get("horizontal_min_angle", 0)
    detection_app.state.user_data.camera_controller.camera_deplacement.horizontal_max_angle = config.get("camera_movement", {}).get("horizontal_max_angle", 270)
    detection_app.state.user_data.camera_controller.camera_deplacement.vertical_min_angle = config.get("camera_movement", {}).get("vertical_min_angle", 45)
    detection_app.state.user_data.camera_controller.camera_deplacement.vertical_max_angle = config.get("camera_movement", {}).get("vertical_max_angle", 135)

    # Mettre à jour les servos avec les nouvelles configurations
    detection_app.state.user_data.camera_controller.camera_deplacement.servo_horizontal.channel = config.get("servo", {}).get("channel_horizontal", 0)
    detection_app.state.user_data.camera_controller.camera_deplacement.servo_vertical.channel = config.get("servo", {}).get("channel_vertical", 1)
    detection_app.state.user_data.camera_controller.camera_deplacement.servo_horizontal.set_servo_angle(detection_app.state.user_data.camera_controller.camera_deplacement.servo_horizontal.current_angle)
    detection_app.state.user_data.camera_controller.camera_deplacement.servo_vertical.set_servo_angle(detection_app.state.user_data.camera_controller.camera_deplacement.servo_vertical.current_angle)

    # Appliquer immédiatement les changements si nécessaire
    logger.info("Configuration rechargée et appliquée.")

# =============================
# =============== MAIN ===================
# ========================================
class DetectionApp:
    """
    Classe principale pour gérer l'application de détection.
    """

    def __init__(self) -> None:
        self.state: DetectionAppState = DetectionAppState()
        self.state.user_data = UserAppCallback()
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

        # Mettre à jour l'activation des mouvements de la caméra
        CAMERA_MOVEMENT_ENABLE = new_config.get("camera_movement", {}).get("enable", True)
        if self.state.user_data.camera_controller:
            self.state.user_data.camera_controller.set_enable_movement = CAMERA_MOVEMENT_ENABLE


        # Mettre à jour les PID
        self.state.user_data.camera_controller.camera_deplacement.pid_x.kp = new_config.get("pid", {}).get("horizontal", {}).get("kp", 1.0)
        self.state.user_data.camera_controller.camera_deplacement.pid_x.ki = new_config.get("pid", {}).get("horizontal", {}).get("ki", 0.0)
        self.state.user_data.camera_controller.camera_deplacement.pid_x.kd = new_config.get("pid", {}).get("horizontal", {}).get("kd", 0.0)

        self.state.user_data.camera_controller.camera_deplacement.pid_y.kp = new_config.get("pid", {}).get("vertical", {}).get("kp", 1.0)
        self.state.user_data.camera_controller.camera_deplacement.pid_y.ki = new_config.get("pid", {}).get("vertical", {}).get("ki", 0.0)
        self.state.user_data.camera_controller.camera_deplacement.pid_y.kd = new_config.get("pid", {}).get("vertical", {}).get("kd", 0.0)

        # Mettre à jour les seuils de zone morte
        self.state.user_data.dead_zone = new_config.get("dead_zone", 0.05)

        # Mettre à jour les angles min/max
        self.state.user_data.camera_controller.camera_deplacement.horizontal_min_angle = new_config.get("camera_movement", {}).get("horizontal_min_angle", 0)
        self.state.user_data.camera_controller.camera_deplacement.horizontal_max_angle = new_config.get("camera_movement", {}).get("horizontal_max_angle", 270)
        self.state.user_data.camera_controller.camera_deplacement.vertical_min_angle = new_config.get("camera_movement", {}).get("vertical_min_angle", 45)
        self.state.user_data.camera_controller.camera_deplacement.vertical_max_angle = new_config.get("camera_movement", {}).get("vertical_max_angle", 135)

        # Mettre à jour les servos avec les nouvelles configurations
        self.state.user_data.camera_controller.camera_deplacement.servo_horizontal.channel = new_config.get("servo", {}).get("channel_horizontal", 0)
        self.state.user_data.camera_controller.camera_deplacement.servo_vertical.channel = new_config.get("servo", {}).get("channel_vertical", 1)
        self.state.user_data.camera_controller.camera_deplacement.servo_horizontal.set_servo_angle(self.state.user_data.camera_controller.camera_deplacement.servo_horizontal.current_angle)
        self.state.user_data.camera_controller.camera_deplacement.servo_vertical.set_servo_angle(self.state.user_data.camera_controller.camera_deplacement.servo_vertical.current_angle)

        logger.info("Configuration mise à jour dynamiquement.")

    def run(self) -> None:
        # Enregistrer les gestionnaires de signaux
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        logger.info("Gestionnaires de signaux enregistrés.")

        # Thread pour déplacer la fenêtre vidéo
        window_mover_thread = threading.Thread(target=self.move_window_thread, daemon=True)
        window_mover_thread.start()
        logger.info("Thread de déplacement de la fenêtre démarré.")

        # Initialiser le déplacement de la caméra
        camera_deplacement = CameraDeplacement(
            p_horizontal=config.get("pid", {}).get("horizontal", {}).get("kp", 30.0),
            i_horizontal=config.get("pid", {}).get("horizontal", {}).get("ki", 0.01),
            d_horizontal=config.get("pid", {}).get("horizontal", {}).get("kd", 0.2),
            p_vertical=config.get("pid", {}).get("vertical", {}).get("kp", 15.0),
            i_vertical=config.get("pid", {}).get("vertical", {}).get("ki", 0.01),
            d_vertical=config.get("pid", {}).get("vertical", {}).get("kd", 0.1),
            dead_zone=DEAD_ZONE
        )
        camera_deplacement.position_zero()
        logger.info("Déplacement de la caméra initialisé et positionné à zéro.")

        # Initialiser le TrackSelector et le CameraController
        self.state.user_data.track_selector = TrackSelector()
        self.state.user_data.camera_controller = CameraController(camera_deplacement)
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

# ========================================
# ======= APPLICATION CALLBACK ========
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
            user_data.camera_controller.update_info(selected_center, time_since_update)
        else:
            user_data.camera_controller.update_info(None, None)

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
        cx_norm = round(center_x, 4)
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
