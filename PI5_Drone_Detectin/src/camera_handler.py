import cv2
import time
import yaml
from picamera2 import Picamera2
from utils.logger import logger


class CameraHandler:
    def __init__(self, config_path='config/config.yaml'):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        cam_cfg = self.config['camera']

        self.width = cam_cfg['resolution_width']
        self.height = cam_cfg['resolution_height']

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={
                "size": (self.width, self.height),
                "format": "RGB888"
            }
        )
        self.picam2.configure(config)
        self.picam2.start()

        time.sleep(2)  # sensor warm-up

        self.last_frame_time = None
        self.last_dt = 0.0

        logger.info(f"Camera started: {self.width}x{self.height}")

    def capture_frame(self):
        """
        Returns:
            frame (np.ndarray)
            timestamp (float)
            dt (float) : time since last frame
        """
        frame = self.picam2.capture_array()
        now = time.time()

        if self.last_frame_time is None:
            dt = 0.0
        else:
            dt = now - self.last_frame_time

        self.last_frame_time = now
        self.last_dt = dt

        return frame, now, dt

    def get_frame_center(self):
        return (self.width // 2, self.height // 2)

    def get_resolution(self):
        return (self.width, self.height)

    def cleanup(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
        logger.info("Camera stopped")
