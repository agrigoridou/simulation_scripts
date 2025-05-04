import airsim
import numpy as np
import time
import math

class AutonomousNavigation:
    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        self.car_state = self.client.getCarState()
        print("Connected!")

        # Ορισμός δρόμου label
        self.road_label_id = 6  # Αν δεν δουλεύει, θα δεις στην κονσόλα το πραγματικό

        # Διαδρομή σε (x, y)
        self.path = [
            (50, 0),
            (50, 50),
            (0, 50),
            (0, 0)
        ]

    def move_forward(self, duration=1, speed=5):
        self.car_controls.throttle = 0.5
        self.car_controls.steering = 0
        self.client.setCarControls(self.car_controls)
        time.sleep(duration)
        self.car_controls.throttle = 0
        self.client.setCarControls(self.car_controls)

    def turn(self, direction, duration=1):
        self.car_controls.throttle = 0.3
        if direction == 'left':
            self.car_controls.steering = -1
        else:
            self.car_controls.steering = 1
        self.client.setCarControls(self.car_controls)
        time.sleep(duration)
        self.car_controls.steering = 0
        self.client.setCarControls(self.car_controls)

    def get_segmentation_center_label(self, window=20):
        # Ζήτα RGB εικόνα από segmentation camera
        resp = self.client.simGetImages([
            airsim.ImageRequest(0, airsim.ImageType.Segmentation, pixels_as_float=False, compress=False)
        ])[0]

        img_rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        try:
            img_rgb = img_rgb.reshape((resp.height, resp.width, 3))
        except ValueError:
            print(f"⚠️ Invalid image shape: {len(img_rgb)} bytes, expected {(resp.height, resp.width, 3)}")
            return -1

        # Υπολογισμός label από RGB
        img_label = img_rgb[:, :, 0] + img_rgb[:, :, 1]*256 + img_rgb[:, :, 2]*256*256

        # Κέντρο εικόνας
        cy, cx = resp.height // 2, resp.width // 2
        w2 = window // 2
        center = img_label[cy - w2:cy + w2, cx - w2:cx + w2]

        labels, counts = np.unique(center, return_counts=True)
        dominant_label = labels[np.argmax(counts)]

        print(f"Detected center label: {dominant_label}")
        return dominant_label

    def drive_to(self, target_x, target_y):
        while True:
            car_state = self.client.getCarState()
            pos = car_state.kinematics_estimated.position
            x, y = pos.x_val, pos.y_val
            distance = math.sqrt((target_x - x)**2 + (target_y - y)**2)

            if distance < 3:
                print(f"Reached waypoint ({target_x}, {target_y})")
                break

            label = self.get_segmentation_center_label(window=20)

            if label == self.road_label_id:
                self.move_forward(duration=0.5)
            else:
                print("⚠️ Off-road detected. Adjusting...")
                self.turn('left', duration=0.3)
                self.move_forward(duration=0.3)

    def run(self):
        for i, (x, y) in enumerate(self.path):
            print(f"\n→ WP {i+1}/{len(self.path)}: ({x},{y})")
            self.drive_to(x, y)

        self.client.enableApiControl(False)
        print("Finished path!")

if __name__ == '__main__':
    nav = AutonomousNavigation()
    nav.run()
