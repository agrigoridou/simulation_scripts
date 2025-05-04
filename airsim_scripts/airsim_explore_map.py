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

    def is_on_road(self, window=20):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        resp = responses[0]

        if resp.width == 0 or resp.height == 0:
            print("Empty image received.")
            return False

        try:
            img_rgb = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width, 3)
        except ValueError:
            print("Σφάλμα στην επεξεργασία εικόνας.")
            return False

        img_labels = img_rgb[:, :, 0]
        h, w = img_labels.shape
        cx, cy = w // 2, h // 2
        half = window // 2
        center_patch = img_labels[cy - half:cy + half, cx - half:cx + half]

        road_pixels = np.count_nonzero(center_patch == 0)
        total_pixels = center_patch.size

        ratio = road_pixels / total_pixels
        return ratio > 0.8

    def drive_forward_smart(self, duration=10):
        self.car_controls.throttle = 0.4
        self.car_controls.steering = 0.0
        self.client.setCarControls(self.car_controls)

        start_time = time.time()

        while time.time() - start_time < duration:
            if self.is_on_road():
                self.car_controls.throttle = 0.5
                self.car_controls.steering = 0.0
                print("On road")
            else:
                print("Off road! Adjusting...")
                self.car_controls.steering += 0.15
                if self.car_controls.steering > 1:
                    self.car_controls.steering = -1
                self.car_controls.throttle = 0.2

            self.client.setCarControls(self.car_controls)
            time.sleep(0.1)

        self.car_controls.throttle = 0
        self.car_controls.steering = 0
        self.client.setCarControls(self.car_controls)

    def run(self):
        print("Ξεκινάμε να οδηγούμε στον δρόμο...")
        self.drive_forward_smart(duration=60)

if __name__ == '__main__':
    nav = AirSimNavigator()
    nav.run()
