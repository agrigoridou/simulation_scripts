# airsim_explore_map.py
# Waypoint navigation + live viz + segmentation‑based road enforcement

import airsim, time, csv, math, random
import numpy as np
import matplotlib.pyplot as plt

# Το label ID που χρησιμοποιεί το AirSim για το “Road”
ROAD_LABEL = 6  

class AirSimNavigatorSegmentationEnforced:
    def __init__(self, waypoint_file, road_bounds):
        # σύνδεση
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # waypoints
        self.waypoints = []
        with open(waypoint_file) as f:
            for r in csv.DictReader(f):
                self.waypoints.append((float(r['x']), float(r['y'])))

        # road bounding box (για plotting μόνο)
        self.min_x, self.max_x, self.min_y, self.max_y = road_bounds

        # log & plot
        self.log = open("navigate_positions.csv", 'w', newline='')
        self.writer = csv.writer(self.log)
        self.writer.writerow(["t", "x", "y", "z", "state"])
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(self.min_x - 5, self.max_x + 5)
        self.ax.set_ylim(self.min_y - 5, self.max_y + 5)
        self.ax.grid(True)

        # plot bounds & waypoints
        bx = [self.min_x, self.max_x, self.max_x, self.min_x, self.min_x]
        by = [self.min_y, self.min_y, self.max_y, self.max_y, self.min_y]
        self.ax.plot(bx, by, 'k--', label="Bounds")
        wp_x, wp_y = zip(*self.waypoints)
        self.ax.plot(wp_x, wp_y, 'bo', label="Waypoints")
        self.path_line, = self.ax.plot([], [], 'g-', label="Path")

    def get_state(self):
        s = self.client.getCarState().kinematics_estimated
        return s.position, s.linear_velocity

    def get_segmentation_center_label(self, window=20):
        # Τραβάμε segmentation εικόνα (labels, όχι RGB)
        resp = self.client.simGetImages([airsim.ImageRequest(
            0, airsim.ImageType.Segmentation, pixels_as_float=False, compress=False
        )])[0]

        # Αποκωδικοποιούμε τα bytes σε numpy πίνακα (1 κανάλι)
        img = np.frombuffer(resp.image_data_uint8, dtype=np.uint8).reshape(resp.height, resp.width)

        # Κεντρικό παράθυρο για ανάλυση label
        cy, cx = resp.height // 2, resp.width // 2
        w2 = window // 2
        center = img[cy - w2:cy + w2, cx - w2:cx + w2]

        # Επιστρέφουμε το πιο συχνό label
        labels, counts = np.unique(center, return_counts=True)
        return labels[np.argmax(counts)]

    def detect_obstacle_depth(self, thr=5.0):
        # Depth-based εμπόδια (προαιρετικό)
        resp = self.client.simGetImages([airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])[0]
        depth = airsim.get_pfm_array(resp)
        h, w = depth.shape
        cy, cx = h // 2, w // 2
        win = 20
        center = depth[cy - win // 2:cy + win // 2, cx - win // 2:cx + win // 2]
        return float(np.min(center)) < thr

    def drive_to(self, tx, ty):
        t0 = time.time()
        state = "DRIVE"
        phase = 0
        path_x, path_y = [], []

        while time.time() - t0 < 40:
            pos, vel = self.get_state()
            x, y = pos.x_val, pos.y_val
            ts = time.time()

            # Έλεγχος με βάση segmentation
            label = self.get_segmentation_center_label(window=20)
            if label != ROAD_LABEL:
                state = "RECOVER"
                # Όπισθεν + στροφή προς κέντρο bounds
                cx = (self.min_x + self.max_x) / 2
                cy = (self.min_y + self.max_y) / 2
                ang = math.atan2(cy - y, cx - x)
                self.controls.throttle = -0.3
                self.controls.steering = max(-1, min(1, ang))
                self.client.setCarControls(self.controls)
                print(f"[{ts:.1f}] OFF-ROAD(seg) → Recovering")
            else:
                # Κανονική οδήγηση
                if state == "DRIVE":
                    ang = math.atan2(ty - y, tx - x)
                    head = math.atan2(vel.y_val, vel.x_val)
                    err = math.atan2(math.sin(ang - head), math.cos(ang - head))
                    self.controls.steering = max(-1, min(1, err))
                    self.controls.throttle = 0.3
                    self.client.setCarControls(self.controls)
                    if self.detect_obstacle_depth(thr=4.0):
                        state = "BACK"
                        phase = ts
                        self.controls.throttle = 0
                        self.client.setCarControls(self.controls)
                elif state == "BACK":
                    if ts - phase < 1.0:
                        self.controls.throttle = -0.3
                        self.controls.steering = 0
                        self.client.setCarControls(self.controls)
                    else:
                        state = "TURN"
                        phase = ts
                        self.controls.throttle = 0
                        self.controls.steering = random.choice([-0.7, 0.7])
                        self.client.setCarControls(self.controls)
                elif state == "TURN":
                    if ts - phase < 1.0:
                        self.client.setCarControls(self.controls)
                    else:
                        state = "DRIVE"
                        time.sleep(0.2)

            # Καταγραφή & οπτικοποίηση
            path_x.append(x)
            path_y.append(y)
            self.path_line.set_data(path_x, path_y)
            self.ax.plot(x, y, 'ro')
            plt.pause(0.01)
            self.writer.writerow([ts, x, y, pos.z_val, state])

            # Άφιξη
            if state == "DRIVE" and math.hypot(tx - x, ty - y) < 1.5:
                print(f"Reached ({tx:.1f},{ty:.1f})")
                break

            time.sleep(0.1)

        # Σταμάτημα
        self.controls.throttle = 0
        self.controls.steering = 0
        self.client.setCarControls(self.controls)

    def run(self):
        for i, (x, y) in enumerate(self.waypoints):
            print(f"→ WP {i + 1}/{len(self.waypoints)}: ({x},{y})")
            self.drive_to(x, y)
        self.log.close()
        plt.legend()
        plt.show()

if __name__ == "__main__":
    # bounds μόνο για plot
    bounds = (0, 50, 0, 10)
    nav = AirSimNavigatorSegmentationEnforced("waypoints.csv", bounds)
    nav.run()
