import airsim
import numpy as np
import time
import math

class AirSimNavigator:
    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        self.client.setCarControls(self.car_controls)
        print("Connected!")

    def get_segmentation_center_label(self, window=20):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        resp = responses[0]

        if resp.width == 0 or resp.height == 0:
            print("Empty image received.")
            return -1

        try:
            img_rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width, 3)
        except ValueError:
            print(f"Image shape from AirSim: {resp.height}x{resp.width}, data size: {len(resp.image_data_uint8)} bytes")
            print(f"Μη αναμενόμενο μέγεθος εικόνας: {len(resp.image_data_uint8)} bytes, target shape=({resp.height},{resp.width})")
            print("Σφάλμα λήψης segmentation εικόνας, σταματάμε.")
            return -1

        img_labels = img_rgb[:, :, 0]

        h, w = img_labels.shape
        cx, cy = w // 2, h // 2
        half_window = window // 2
        center_pixels = img_labels[cy - half_window:cy + half_window, cx - half_window:cx + half_window].flatten()

        dominant_label = int(np.bincount(center_pixels).argmax())
        print(f"Detected center label: {dominant_label}")
        return dominant_label

    def drive_to(self, x, y):
        position = self.client.getCarState().kinematics_estimated.position
        current_x = position.x_val
        current_y = position.y_val

        dx = x - current_x
        dy = y - current_y
        angle = math.atan2(dy, dx)

        speed = 5
        duration = math.hypot(dx, dy) / speed

        self.car_controls.throttle = 0.5
        self.car_controls.steering = 0.0
        self.client.setCarControls(self.car_controls)

        start_time = time.time()

        while time.time() - start_time < duration:
            label = self.get_segmentation_center_label(window=20)
            if label != 0:
                print("Off-road detected. Adjusting...")
                self.car_controls.steering += 0.1
                self.client.setCarControls(self.car_controls)
            time.sleep(0.1)

        self.car_controls.throttle = 0.0
        self.car_controls.steering = 0.0
        self.client.setCarControls(self.car_controls)
        time.sleep(1)

    def run(self):
        waypoints = [(50, 0), (50, 50), (0, 50), (0, 0)]
        for i, (x, y) in enumerate(waypoints):
            print(f"\n→ WP {i+1}/{len(waypoints)}: ({x},{y})")
            self.drive_to(x, y)

if __name__ == '__main__':
    nav = AirSimNavigator()
    nav.run()
