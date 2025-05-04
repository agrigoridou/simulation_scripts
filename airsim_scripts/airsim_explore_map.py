import airsim
import numpy as np
import time

class RoadFollower:
    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()
        print("Connected to AirSim.")

    def get_segmentation_image(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
        return img_rgb[:, :, 0]  # παίρνουμε μόνο το label channel

    def follow_road(self, duration=60):
        start = time.time()
        while time.time() - start < duration:
            try:
                seg_img = self.get_segmentation_image()
                h, w = seg_img.shape
                scan_y = int(h * 0.75)  # πιο κάτω στη φωτογραφία
                line = seg_img[scan_y]

                road_indices = np.where(line == 0)[0]  # label 0 = δρόμος

                if len(road_indices) == 0:
                    print("No road detected, slowing down.")
                    self.controls.throttle = 0.2
                    self.controls.steering = 0
                else:
                    road_center = int(np.mean(road_indices))
                    image_center = w // 2
                    error = (road_center - image_center) / (w // 2)  # κανονικοποίηση [-1, 1]

                    # Προσαρμογή τιμονιού
                    self.controls.steering = np.clip(error * 1.0, -1, 1)
                    self.controls.throttle = 0.5

                    print(f"Steering: {self.controls.steering:.2f}, Error: {error:.2f}")

                self.client.setCarControls(self.controls)
                time.sleep(0.1)
            except Exception as e:
                print("Σφάλμα:", e)
                self.controls.throttle = 0
                self.client.setCarControls(self.controls)

        self.controls.throttle = 0
        self.client.setCarControls(self.controls)
        print("Τέλος διαδρομής.")

    def run(self):
        print("Ξεκινώ να ακολουθώ τον δρόμο...")
        self.follow_road()

if __name__ == '__main__':
    bot = RoadFollower()
    bot.run()
