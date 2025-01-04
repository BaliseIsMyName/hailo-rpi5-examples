import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import argparse
import multiprocessing
import numpy as np
import setproctitle
import cv2
import time
import hailo
from hailo_rpi_common import (
    get_default_parser,
    QUEUE,
    SOURCE_PIPELINE,
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    USER_CALLBACK_PIPELINE,
    DISPLAY_PIPELINE,
    GStreamerApp,
    app_callback_class,
    dummy_callback,
    detect_hailo_arch,
)



# -----------------------------------------------------------------------------------------------
# User Gstreamer Application
# -----------------------------------------------------------------------------------------------

# This class inherits from the hailo_rpi_common.GStreamerApp class
class GStreamerDetectionApp(GStreamerApp):
    def __init__(self, app_callback, user_data):
        parser = get_default_parser()
        parser.add_argument(
            "--labels-json",
            default=None,
            help="Path to costume labels JSON file",
        )
        args = parser.parse_args()
        # Call the parent class constructor
        super().__init__(args, user_data)
        # Additional initialization code can be added here
        # Set Hailo parameters these parameters should be set based on the model used
        self.batch_size = 2
        self.network_width = 640
        self.network_height = 640
        self.network_format = "RGB"
        nms_score_threshold = 0.3
        nms_iou_threshold = 0.45


        # Determine the architecture if not specified
        if args.arch is None:
            detected_arch = detect_hailo_arch()
            if detected_arch is None:
                raise ValueError("Could not auto-detect Hailo architecture. Please specify --arch manually.")
            self.arch = detected_arch
            print(f"Auto-detected Hailo architecture: {self.arch}")
        else:
            self.arch = args.arch


        if args.hef_path is not None:
            self.hef_path = args.hef_path
        # Set the HEF file path based on the arch
        elif self.arch == "hailo8":
            self.hef_path = os.path.join(self.current_path, '../resources/yolov8m.hef')
        else:  # hailo8l
            self.hef_path = os.path.join(self.current_path, '../resources/yolov8s_h8l.hef')

        # Set the post-processing shared object file
        self.post_process_so = os.path.join(self.current_path, '../resources/libyolo_hailortpp_postprocess.so')

        # User-defined label JSON file
        self.labels_json = args.labels_json

        self.app_callback = app_callback

        self.thresholds_str = (
            f"nms-score-threshold={nms_score_threshold} "
            f"nms-iou-threshold={nms_iou_threshold} "
            f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )

        # Set the process title
        setproctitle.setproctitle("Hailo Detection App")

        self.create_pipeline()

    def get_pipeline_string(self):
        source_pipeline = SOURCE_PIPELINE(self.video_source)
        detection_pipeline = INFERENCE_PIPELINE(
            hef_path=self.hef_path,
            post_process_so=self.post_process_so,
            batch_size=self.batch_size,
            config_json=self.labels_json,
            additional_params=self.thresholds_str)
        user_callback_pipeline = USER_CALLBACK_PIPELINE()
        display_pipeline = DISPLAY_PIPELINE(video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps)
        pipeline_string = (
            f'{source_pipeline} '
            f'{detection_pipeline} ! '
            f'{user_callback_pipeline} ! '
                        # hailooverlay
            f'{QUEUE("hailo_display_hailooverlay_q")} ! '
            f'hailooverlay name=hailo_overlay ! '
            # videoconvert pour assurer format compatible cairooverlay
            f'{QUEUE("pre_cairo_convert_q")} ! '
            f'videoconvert name=pre_cairo_convert ! '
            # cairooverlay
            f'{QUEUE("cairo_q")} ! '
            f'cairooverlay name=cairo_overlay ! '
            # videoconvert après cairooverlay
            f'{QUEUE("hailo_display_videoconvert_q")} ! '
            f'videoconvert name=hailo_display_videoconvert n-threads=2 qos=false ! '
            # Ajout du tee
            f'tee name=t '
            # # Branche pour l'affichage local
            f't. ! queue ! '
            f'fpsdisplaysink name=hailo_display video-sink={self.video_sink} sync={self.sync} text-overlay={self.show_fps} signal-fps-measurements=true '
            # Branche pour le streaming RTMP
            f't. ! queue ! '
            f'x264enc tune=zerolatency bitrate=1200 speed-preset=superfast ! '
            f'flvmux streamable=true ! '
            f'rtmpsink location="rtmp://localhost/live/stream live=1" '
        )
        print(pipeline_string)
        return pipeline_string

# if __name__ == "__main__":
#     # Create an instance of the user app callback class
#     user_data = app_callback_class()
#     app_callback = dummy_callback
#     app = GStreamerDetectionApp(app_callback, user_data)
#     app.run()
