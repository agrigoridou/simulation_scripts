import airsim
import numpy as np
import time
import math

class AirSimNavigator:
    def __init__(self):
        # Σύνδεση με τον εξομοιωτή
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        self.client.simSetSegmentationObjectID("[\w]*", 0, True)  # Επαναφορά όλων των αντικειμένων σε ID 0
        self.client.simSetSegmentationObjectID("Road.*", 1, True)  # Ορισμός "Road" σε label ID 1
        print("Connected!")

    def get_segmentation_center_label(self, window=20):
        # Λήψη εικόνας από την κάμερα segmentation
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        resp = responses[0]

        if resp.width == 0 or resp.height == 0:
            print("Empty image received.")
            return -1

        img = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width)
        
        # Απόκτηση των pixel γύρω από το κέντρο της εικόνας
        h, w = img.shape
        cx, cy = w // 2, h // 2
        half_window = window // 2
        center_pixels = img[cy - half_window:cy + half_window, cx - half_window:cx + half_window].flatten()

        # Εύρεση του επικρατέστερου label
        dominant_label = int(np.bincount(center_pixels).argmax())
        print(f"Detected center label: {dominant_label}")  # 👈 Προστέθηκε
        return dominant_label

    def drive_to(self, x, y, speed=5):
        pos = self.client.getCarState().kinematics_estimated.position
        start_x = pos.x_val
        start_y = pos.y_val

        dx = x - start_x
        dy = y - start_y
        distance = math.hypot(dx, dy)

        angle = math.atan2(dy, dx)
        self.car_controls.throttle = 0.5
        self.car_controls.steering = 0.0
        self.client.setCarControls(self.car_controls)

        duration = distance / speed
        start_time = time.time()

        while time.time() - start_time < duration:
            label = self.get_segmentation_center_label(window=20)

            if label != 1:
                print("Off-road detected. Adjusting...")
                self.car_controls.steering += 0.1  # απλή προσαρμογή
            else:
                self.car_controls.steering *= 0.9  # Επαναφορά πορείας

            self.client.setCarControls(self.car_controls)
            time.sleep(0.1)

        # Σταμάτημα του αυτοκινήτου
        self.car_controls.throttle = 0
        self.car_controls.steering = 0
        self.client.setCarControls(self.car_controls)
        print("Reached waypoint.")

    def run(self):
        # Ορισμός κάποιων σημείων που θα επισκεφτεί
        waypoints = [(50, 0), (50, 50), (0, 50), (0, 0)]
        for i, (x, y) in enumerate(waypoints):
            print(f"\n→ WP {i+1}/{len(waypoints)}: ({x},{y})")
            self.drive_to(x, y)

if __name__ == "__main__":
    nav = AirSimNavigator()
    nav.run()
