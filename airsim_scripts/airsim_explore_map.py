import airsim
import numpy as np
import time
import math

class RoadNavigator:
    def __init__(self, client, waypoints):
        self.client = client
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.car.enableApiControl(True)
        self.client.car.armDisarm(True)
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

    def drive_to_waypoint(self, x, y):
        current = self.client.getCarState().kinematics_estimated.position
        z = current.z_val
        target = airsim.Vector3r(x, y, z)
        self.client.moveToPositionAsync(x, y, z, velocity=5).join()

    def run(self):
        print("Ξεκινάμε πλοήγηση...")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            self.drive_to_waypoint(x, y)

            while True:
                label = self.get_segmentation_center_label()
                if label is None:
                    print("Σφάλμα κατά την ανάγνωση ετικέτας.")
                    break

                if label != 0:
                    print(f"Βγήκαμε εκτός δρόμου! Ετικέτα: {label}")
                    self.client.car.brake()
                    return
                else:
                    print(f"Είμαστε στον δρόμο. Ετικέτα: {label}")
                time.sleep(0.5)

        print("Η πλοήγηση ολοκληρώθηκε.")

if __name__ == "__main__":
    client = airsim.CarClient()
    client.confirmConnection()

    # === Δώσε χειροκίνητα τα waypoints πάνω στον δρόμο ===
    # (x, y) συντεταγμένες σε AirSim World
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
