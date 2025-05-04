import airsim
import numpy as np
import time
import math

class RoadNavigator:
    def __init__(self, client, waypoints):
        self.client = client
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.waypoints = waypoints
        self.client.simSetSegmentationObjectID("[Car]", 0, True)
        self.client.simSetSegmentationObjectID("Road*", 0, True)  # Label 0 = δρόμος

    def get_heading(self):
        orientation = self.client.getCarState().kinematics_estimated.orientation
        yaw = airsim.to_eularian_angles(orientation)[2]
        return yaw

    def get_segmentation_center_label(self, window=10):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        if not responses or len(responses) == 0:
            print("Σφάλμα λήψης εικόνας.")
            return None

        resp = responses[0]
        img = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        try:
            img = img.reshape((resp.height, resp.width, 3))
        except:
            print("Σφάλμα στο reshape εικόνας.")
            return None

        h, w = resp.height, resp.width
        center_x = w // 2
        center_y = h // 2
        patch = img[center_y - window:center_y + window, center_x - window:center_x + window]
        labels = patch[:, :, 0]
        unique, counts = np.unique(labels, return_counts=True)
        dominant_label = unique[np.argmax(counts)]

        print(f"Detected center label: {dominant_label}")
        return dominant_label

    def drive_to_waypoint(self, x, y, tolerance=2.0):
        while True:
            pos = self.client.getCarState().kinematics_estimated.position
            dx = x - pos.x_val
            dy = y - pos.y_val
            distance = math.sqrt(dx**2 + dy**2)

            if distance < tolerance:
                break

            heading = math.atan2(dy, dx)
            car_heading = self.get_heading()
            steer = heading - car_heading
            steer = max(-1.0, min(1.0, steer))

            controls = airsim.CarControls()
            controls.throttle = 0.5
            controls.steering = steer
            self.client.setCarControls(controls)

            label = self.get_segmentation_center_label()
            if label is None or label != 0:
                print(f"Βγήκαμε εκτός δρόμου! Ετικέτα: {label}")
                controls = airsim.CarControls()
                controls.throttle = 0.0
                controls.brake = 1.0
                self.client.setCarControls(controls)
                return False

            time.sleep(0.1)

        controls = airsim.CarControls()
        controls.throttle = 0.0
        controls.brake = 1.0
        self.client.setCarControls(controls)
        return True

    def run(self):
        print("Ξεκινάμε πλοήγηση...")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            ok = self.drive_to_waypoint(x, y)
            if not ok:
                print("Πλοήγηση διακόπηκε λόγω εξόδου από δρόμο.")
                break
        print("Πλοήγηση ολοκληρώθηκε ή διεκόπη.")

if __name__ == "__main__":
    client = airsim.CarClient()
    client.confirmConnection()

    # Waypoints πάνω στον δρόμο (ενδεικτικά, προσαρμόστε αναλόγως τον χάρτη σας)
    waypoints = [
        (0, 0),
        (30, 0),
        (60, 0),
        (60, 30),
        (60, 60),
        (30, 60),
        (0, 60),
        (0, 30),
        (0, 0)
    ]

    nav = RoadNavigator(client, waypoints)
    nav.run()
