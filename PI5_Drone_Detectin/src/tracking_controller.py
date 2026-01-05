import yaml
from utils.kalman_filter import KalmanFilter2D
from utils.pid_controller import PIDController
from utils.logger import logger


class TrackingController:
    def __init__(self, servo_controller, camera_handler, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.servo = servo_controller
        self.camera = camera_handler

        self.kalman = KalmanFilter2D(config_path)
        self.pid_pan = PIDController("pan", config_path)
        self.pid_tilt = PIDController("tilt", config_path)

        self.frame_w, self.frame_h = self.camera.get_resolution()

        logger.info("TrackingController initialized")

    def reset(self):
        self.kalman.reset()
        self.pid_pan.reset()
        self.pid_tilt.reset()

    def update(self, target_center, dt):
        """
        target_center: (x, y) or None
        dt: frame delta time
        """
        if target_center is None:
            return

        tx, ty = target_center
        cx, cy = self.camera.get_frame_center()

        # Normalized error (-1 to +1)
        err_x = (tx - cx) / cx
        err_y = (ty - cy) / cy

        # Predict + update Kalman
        self.kalman.predict(dt)
        fx, fy = self.kalman.update(err_x, err_y)

        # PID control
        pan_delta = self.pid_pan.compute(fx, dt)
        tilt_delta = self.pid_tilt.compute(-fy, dt)  # invert Y-axis

        self.servo.apply_delta(pan_delta, tilt_delta)
