import yaml
import platform
from utils.logger import logger

# Optional GPIO import (only works on Pi)
IS_PI = platform.system() == "Linux"

if IS_PI:
    import pigpio


class ServoController:
    def __init__(self, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        pt = cfg["pan_tilt"]

        self.pan_min = pt["pan_min"]
        self.pan_max = pt["pan_max"]
        self.tilt_min = pt["tilt_min"]
        self.tilt_max = pt["tilt_max"]

        self.pan_angle = pt["pan_home"]
        self.tilt_angle = pt["tilt_home"]

        # GPIO pins (change later if needed)
        self.pan_pin = pt.get("pan_gpio", 18)
        self.tilt_pin = pt.get("tilt_gpio", 19)

        self.pi = None
        if IS_PI:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio not running")

            self.pi.set_mode(self.pan_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.tilt_pin, pigpio.OUTPUT)

            self._write_servo(self.pan_pin, self.pan_angle)
            self._write_servo(self.tilt_pin, self.tilt_angle)

        logger.info(
            f"ServoController initialized (PAN={self.pan_angle}, TILT={self.tilt_angle})"
        )

    def _angle_to_pulsewidth(self, angle):
        """
        Maps 0–270° → 500–2500 µs (adjust later during calibration)
        """
        return int(500 + (angle / 270.0) * 2000)

    def _write_servo(self, pin, angle):
        pw = self._angle_to_pulsewidth(angle)
        self.pi.set_servo_pulsewidth(pin, pw)

    def apply_delta(self, pan_delta=0.0, tilt_delta=0.0):
        new_pan = self.pan_angle + pan_delta
        new_tilt = self.tilt_angle + tilt_delta

        # Clamp limits
        new_pan = max(self.pan_min, min(self.pan_max, new_pan))
        new_tilt = max(self.tilt_min, min(self.tilt_max, new_tilt))

        self.pan_angle = new_pan
        self.tilt_angle = new_tilt

        if IS_PI:
            self._write_servo(self.pan_pin, self.pan_angle)
            self._write_servo(self.tilt_pin, self.tilt_angle)

        logger.debug(
            f"Servo move → PAN={self.pan_angle:.2f}, TILT={self.tilt_angle:.2f}"
        )

    def center(self):
        self.pan_angle = (self.pan_min + self.pan_max) / 2
        self.tilt_angle = (self.tilt_min + self.tilt_max) / 2

        if IS_PI:
            self._write_servo(self.pan_pin, self.pan_angle)
            self._write_servo(self.tilt_pin, self.tilt_angle)

    def shutdown(self):
        if IS_PI and self.pi:
            self.pi.set_servo_pulsewidth(self.pan_pin, 0)
            self.pi.set_servo_pulsewidth(self.tilt_pin, 0)
            self.pi.stop()

        logger.info("ServoController shutdown")
