import time
import cv2

from src.camera_handler import CameraHandler
from src.hailo_detector import Detector
from src.target_manager import TargetManager
from src.servo_controller import ServoController
from src.scanner import Scanner
from src.tracking_controller import TrackingController
from utils.logger import logger


def main():
    logger.info("Starting Drone EO Tracking System")

    camera = CameraHandler()
    detector = Detector()
    target_manager = TargetManager()
    servo = ServoController()
    scanner = Scanner(servo)
    tracker = TrackingController(servo, camera)

    state = "SCAN"

    try:
        while True:
            frame, timestamp, dt = camera.capture_frame()

            detections = detector.detect(frame, timestamp)
            target_center, locked = target_manager.update(detections)

            if state == "SCAN":
                scanner.update()

                if locked:
                    logger.info("Switching to TRACK mode")
                    tracker.reset()
                    state = "TRACK"

            elif state == "TRACK":
                tracker.update(target_center, dt)

                if not locked:
                    logger.info("Target lost → returning to SCAN")
                    tracker.reset()
                    state = "SCAN"

            # ---- OPTIONAL DISPLAY (DEV MODE) ----
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cx, cy = camera.get_frame_center()
            cv2.drawMarker(
                frame, (cx, cy), (0, 0, 255),
                markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2
            )

            cv2.putText(
                frame,
                f"MODE: {state}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2
            )

            cv2.imshow("EO Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        logger.info("Interrupted by user")

    finally:
        logger.info("Shutting down system")
        servo.shutdown()
        camera.cleanup()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
