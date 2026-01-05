import yaml
import time
from utils.logger import logger


class Scanner:
    def __init__(self, servo_controller, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.servo = servo_controller

        scan_cfg = cfg["scan"]
        pt_cfg = cfg["pan_tilt"]

        self.pan_min = pt_cfg["pan_min"]
        self.pan_max = pt_cfg["pan_max"]

        self.scan_speed = scan_cfg["pan_speed_deg_per_sec"]

        self.direction = 1  # 1 = right, -1 = left
        self.last_time = time.time()

        logger.info("Scanner initialized")

    def update(self):
        """
        Call periodically.
        Moves pan back and forth.
        """
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0:
            return

        delta = self.direction * self.scan_speed * dt
        next_pan = self.servo.pan_angle + delta

        if next_pan >= self.pan_max:
            self.direction = -1
            next_pan = self.pan_max
        elif next_pan <= self.pan_min:
            self.direction = 1
            next_pan = self.pan_min

        self.servo.apply_delta(pan_delta=next_pan - self.servo.pan_angle)
