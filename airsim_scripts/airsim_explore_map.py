# Navigation toward multiple targets with live visualization using Matplotlib and depth‑based obstacle detection
# Method: Waypoint-based navigation with obstacle avoidance state machine

import airsim
import time
import csv
import math
import random
import matplotlib.pyplot as plt
import numpy as np

class AirSimWaypointNavigatorWithAdvancedAvoidance:
    def __init__(self, waypoint_file):
        # connect to AirSim CarClient
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # load waypoints
        self.waypoints = self.load_waypoints(waypoint_file)

        # prepare CSV log
        self.csv_file = open("navigate_positions.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z", "state"])

        # set up live plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("AirSim Navigation – Advanced Obstacle Avoidance")
        self.ax.set_xlabel("X Position");  self.ax.set_ylabel("Y Position")
        self.ax.set_xlim(-10, 60);  self.ax.set_ylim(-10, 60)
        self.ax.grid(True)
        # plot waypoints
        wp_x, wp_y = zip(*self.waypoints)
        self.ax.plot(wp_x, wp_y, 'bo', label="Waypoints")
        self.path_line, = self.ax.plot([], [], 'g-', label="Path")

    def load_waypoints(self, filename):
        waypoints = []
        with open(filename) as f:
            for row in csv.DictReader(f):
                waypoints.append((float(row['x']), float(row['y'])))
        return waypoints

    def get_position(self):
        state = self.client.getCarState()
        return state.kinematics_estimated.position, state.kinematics_estimated.linear_velocity

    def angle_to_target(self, pos, target):
        return math.atan2(target[1] - pos.y_val, target[0] - pos.x_val)

    def current_heading(self, vel):
        return math.atan2(vel.y_val, vel.x_val)

    def detect_obstacle_depth(self, threshold=5.0):
        # get depth image from camera 1
        responses = self.client.simGetImages([airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])
        if not responses:
            return False
        depth = airsim.get_pfm_array(responses[0])
        min_dist = float(np.min(depth))
        return min_dist < threshold

    def drive_to_waypoint(self, tx, ty, tolerance=1.5, timeout=30):
        start = time.time()
        path_x, path_y = [], []
        state = "DRIVE"   # DRIVE, BACKUP, TURN
        phase_start = start

        while time.time() - start < timeout:
            pos, vel = self.get_position()
            ts = time.time()
            dist = math.hypot(tx - pos.x_val, ty - pos.y_val)

            # state machine
            if state == "DRIVE":
                # steer toward target
                desired = self.angle_to_target(pos, (tx, ty))
                heading = self.current_heading(vel)
                err = math.atan2(math.sin(desired - heading), math.cos(desired - heading))
                self.controls.steering = max(-1, min(1, err))
                self.controls.throttle = 0.3
                self.client.setCarControls(self.controls)
                # detect obstacle
                if self.detect_obstacle_depth(threshold=4.0):
                    state = "BACKUP"
                    phase_start = ts
                    self.controls.throttle = 0
                    self.client.setCarControls(self.controls)

            elif state == "BACKUP":
                # reverse for 1s
                if ts - phase_start < 1.0:
                    self.controls.throttle = -0.3
                    self.controls.steering = 0
                    self.client.setCarControls(self.controls)
                else:
                    state = "TURN"
                    phase_start = ts
                    # choose random turn
                    self.controls.throttle = 0
                    self.controls.steering = random.choice([-0.7, 0.7])
                    self.client.setCarControls(self.controls)

            elif state == "TURN":
                # turn in place for 1s
                if ts - phase_start < 1.0:
                    self.client.setCarControls(self.controls)
                else:
                    state = "DRIVE"
                    time.sleep(0.3)  # cooldown before driving

            # log & visualize
            path_x.append(pos.x_val); path_y.append(pos.y_val)
            self.path_line.set_data(path_x, path_y)
            self.ax.plot(pos.x_val, pos.y_val, 'ro')
            plt.pause(0.01)

            self.csv_writer.writerow([ts, pos.x_val, pos.y_val, pos.z_val, state])
            print(f"[{ts:.1f}] to ({tx:.1f},{ty:.1f}) pos=({pos.x_val:.1f},{pos.y_val:.1f}) state={state}")

            if dist < tolerance:
                print(f"Reached ({tx:.1f},{ty:.1f})")
                break

            time.sleep(0.2)

        self.stop()

    def navigate_all(self):
        for i,(x,y) in enumerate(self.waypoints):
            print(f"\n→ Waypoint {i+1}/{len(self.waypoints)}: ({x},{y})")
            self.drive_to_waypoint(x, y)
        self.csv_file.close()
        plt.legend(); plt.show()

    def stop(self):
        self.controls.throttle = 0; self.controls.steering = 0
        self.client.setCarControls(self.controls)

if __name__ == "__main__":
    nav = AirSimWaypointNavigatorWithAdvancedAvoidance("waypoints.csv")
    nav.navigate_all()
