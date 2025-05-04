import airsim
import time
import numpy as np
import math


class SimpleNavigator:
    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        print("Connected!\n")
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

        self.speed = 5  # m/s
        self.stop_distance = 3  # meters
        self.offroad_label = 255  # offroad label in segmentation

    def get_segmentation_center_label(self, window=20):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        resp = responses[0]

        if resp.width == 0 or resp.height == 0:
            print("Empty image received.")
            return -1

        try:
            print(f"Image shape from AirSim: {resp.height}x{resp.width}, data size: {len(resp.image_data_uint8)} bytes")
            img = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width)
        except ValueError:
            print(f"Μη αναμενόμενο μέγεθος εικόνας: {len(resp.image_data_uint8)} bytes, "
                  f"target shape=({resp.height},{resp.width})")
            return -1

        h, w = img.shape
        cx, cy = w // 2, h // 2
        half_window = window // 2
        center_pixels = img[cy - half_window:cy + half_window, cx - half_window:cx + half_window].flatten()

        dominant_label = int(np.bincount(center_pixels).argmax())
        print(f"Detected center label: {dominant_label}")
        return dominant_label

    def is_offroad(self, label):
        return label == self.offroad_label

    def drive_to(self, x, y):
        car_state = self.client.getCarState()
        pos = car_state.kinematics_estimated.position
        dx = x - pos.x_val
        dy = y - pos.y_val
        distance = math.sqrt(dx**2 + dy**2)

        angle = math.atan2(dy, dx)
        throttle = 0.5
        steering = angle - car_state.kinematics_estimated.orientation.z_val
        steering = np.clip(steering, -1, 1)

        self.car_controls.throttle = throttle
        self.car_controls.steering = steering
        self.client.setCarControls(self.car_controls)

        while distance > self.stop_distance:
            label = self.get_segmentation_center_label(window=20)
            if label == -1:
                print("Σφάλμα λήψης segmentation εικόνας, σταματάμε.")
                break

            if self.is_offroad(label):
                print("Off-road detected. Adjusting...")
                self.car_controls.steering += 0.1
                self.client.setCarControls(self.car_controls)

            car_state = self.client.getCarState()
            pos = car_state.kinematics_estimated.position
            dx = x - pos.x_val
            dy = y - pos.y_val
            distance = math.sqrt(dx**2 + dy**2)
            time.sleep(0.5)

        self.stop()

    def stop(self):
        self.car_controls.throttle = 0
        self.car_controls.brake = 1
        self.client.setCarControls(self.car_controls)

    def run(self):
        waypoints = [(50, 0), (50, 50), (0, 50), (0, 0)]
        for i, (x, y) in enumerate(waypoints):
            print(f"\n→ WP {i+1}/{len(waypoints)}: ({x},{y})")
            self.drive_to(x, y)
            time.sleep(1)


if __name__ == '__main__':
    nav = SimpleNavigator()
    nav.run()
