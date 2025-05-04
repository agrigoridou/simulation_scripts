import airsim
import numpy as np
import time
import math

class RoadNavigator:
    def __init__(self, client, waypoints):
        self.client = client
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.waypoints = waypoints

    def get_segmentation_center_label(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        if len(responses) < 1:
            return None

        resp = responses[0]
        img1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)

        try:
            img_rgb = img1d.reshape(resp.height, resp.width, 3)
        except ValueError:
            print(f"Σφάλμα στη λήψη εικόνας: shape={resp.height}x{resp.width}, μέγεθος δεδομένων={len(img1d)}")
            return None

        center_pixel = img_rgb[resp.height // 2, resp.width // 2]
        label = center_pixel[0]
        return label

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
            if label != 0:
                print(f"Βγήκαμε εκτός δρόμου! Ετικέτα: {label}")
                self.client.brake()
                return False

            time.sleep(0.1)

        self.client.brake()
        return True

    def get_heading(self):
        orientation = self.client.getCarState().kinematics_estimated.orientation
        yaw = airsim.to_eularian_angles(orientation)[2]
        return yaw

    def run(self):
        print("Ξεκινάμε πλοήγηση...")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            ok = self.drive_to_waypoint(x, y)
            if not ok:
                print("Διακόπηκε λόγω εξόδου από δρόμο.")
                break
        print("Τέλος πλοήγησης.")

if __name__ == "__main__":
    client = airsim.CarClient()
    client.confirmConnection()

    waypoints = [
        (0, 0),
        (30, 0),
        (60, 0),
        (60, 30),
        (60, 60),
        (30, 60),
        (0, 60),
        (-30, 60),
        (-60, 60),
        (-60, 30),
        (-60, 0)
    ]

    nav = RoadNavigator(client, waypoints)
    nav.run()
