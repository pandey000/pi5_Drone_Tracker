import numpy as np
import yaml


class KalmanFilter2D:
    def __init__(self, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        kcfg = cfg["kalman"]

        self.q = kcfg["process_noise"]
        self.r = kcfg["measurement_noise"]

        # State: [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        # Covariance
        self.P = np.eye(4)

        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self.R = np.eye(2) * self.r
        self.initialized = False

    def reset(self):
        self.x[:] = 0
        self.P = np.eye(4)
        self.initialized = False

    def predict(self, dt):
        if not self.initialized or dt <= 0:
            return

        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        Q = np.eye(4) * self.q

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, meas_x, meas_y):
        z = np.array([[meas_x], [meas_y]])

        if not self.initialized:
            self.x[0, 0] = meas_x
            self.x[1, 0] = meas_y
            self.initialized = True
            return self.x[0, 0], self.x[1, 0]

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return self.x[0, 0], self.x[1, 0]
