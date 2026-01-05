import time
import torch
import yaml
import numpy as np
from utils.logger import logger


class Detector:
    def __init__(self, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        det_cfg = self.config["detection"]

        self.backend = det_cfg["backend"]
        self.model_path = det_cfg["model_path"]
        self.conf_thres = det_cfg["confidence_threshold"]
        self.inference_interval = det_cfg["inference_interval"]

        self.last_inference_time = 0.0
        self.last_detections = []

        if self.backend == "cpu":
            logger.info("Loading YOLOv5 model on CPU")
            self.model = torch.hub.load(
                "ultralytics/yolov5",
                "custom",
                path=self.model_path,
                force_reload=False
            )
            self.model.conf = self.conf_thres
            self.model.eval()
        else:
            raise NotImplementedError("Hailo backend not enabled yet")

    def detect(self, frame, timestamp):
        """
        Returns cached detections if inference is throttled.
        """
        if timestamp - self.last_inference_time < self.inference_interval:
            return self.last_detections

        self.last_inference_time = timestamp

        results = self.model(frame, size=640)
        detections = []

        for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
            x1, y1, x2, y2 = map(int, xyxy)

            detections.append({
                "bbox": (x1, y1, x2, y2),
                "confidence": float(conf),
                "class_id": int(cls)
            })

        self.last_detections = detections
        return detections
