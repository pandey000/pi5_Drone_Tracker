import yaml


class PIDController:
    def __init__(self, axis, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.axis = axis  # "pan" or "tilt"

        pt_cfg = cfg["pan_tilt"]
        trk_cfg = cfg["tracking"]

        self.deadzone = trk_cfg["deadzone"]
        self.max_step = pt_cfg["max_step_deg"]

        # Tunable gains (safe defaults)
        self.kp = 20.0
        self.kd = 4.0

        self.prev_error = 0.0

    def reset(self):
        self.prev_error = 0.0

    def compute(self, error, dt):
        """
        error: normalized (-1 to 1)
        returns: delta angle in degrees
        """
        if abs(error) < self.deadzone:
            return 0.0

        if dt <= 0:
            return 0.0

        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.kd * derivative

        self.prev_error = error

        # Clamp output
        output = max(-self.max_step, min(self.max_step, output))

        return output
