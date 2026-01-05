import yaml
from utils.logger import logger


class TargetManager:
    def __init__(self, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        trk_cfg = self.config["tracking"]

        self.lock_frames_required = trk_cfg["lock_frames"]
        self.lost_frames_allowed = trk_cfg["lost_frames"]

        self.lock_counter = 0
        self.lost_counter = 0

        self.locked = False
        self.current_target = None

    def reset(self):
        self.lock_counter = 0
        self.lost_counter = 0
        self.locked = False
        self.current_target = None

    def _select_closest_target(self, detections):
        """
        Selects detection with largest bbox area.
        """
        best = None
        max_area = 0

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            area = (x2 - x1) * (y2 - y1)

            if area > max_area:
                max_area = area
                best = det

        return best

    def update(self, detections):
        """
        Returns:
            target_center (x, y) or None
            locked (bool)
        """
        if not detections:
            if self.locked:
                self.lost_counter += 1
                if self.lost_counter > self.lost_frames_allowed:
                    logger.info("Target lost → unlocking")
                    self.reset()
            return None, self.locked

        best = self._select_closest_target(detections)

        if not self.locked:
            self.lock_counter += 1
            if self.lock_counter >= self.lock_frames_required:
                self.locked = True
                logger.info("Target locked")
        else:
            self.lost_counter = 0

        self.current_target = best

        x1, y1, x2, y2 = best["bbox"]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        return (cx, cy), self.locked
