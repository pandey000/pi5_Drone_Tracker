from src.servo_controller import ServoController
from src.tracking_controller import TrackingController
import time

# ---- MOCK CAMERA (Windows-safe) ----
class MockCamera:
    def __init__(self, w=1280, h=720):
        self.w = w
        self.h = h

    def get_resolution(self):
        return self.w, self.h

    def get_frame_center(self):
        return self.w // 2, self.h // 2


servo = ServoController()
camera = MockCamera()
tracker = TrackingController(servo, camera)

cx, cy = camera.get_frame_center()

# Simulate target moving right across frame
for i in range(30):
    fake_target = (cx + i * 15, cy)
    tracker.update(fake_target, dt=0.1)
    time.sleep(0.1)

servo.shutdown()
print("Tracking controller mock test complete")
