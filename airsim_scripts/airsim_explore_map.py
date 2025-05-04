# Navigation toward multiple targets: Waypoint-based navigation using AirSim

import airsim
import time
import csv
import math

class AirSimWaypointNavigator:
    def __init__(self, waypoint_file):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        self.waypoints = self.load_waypoints(waypoint_file)

        self.csv_file = open("navigate_positions.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z"])

    def load_waypoints(self, filename):
        waypoints = []
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append((float(row['x']), float(row['y'])))
        return waypoints

    def get_position(self):
        car_state = self.client.getCarState()
        return car_state.kinematics_estimated.position, car_state.kinematics_estimated.linear_velocity

    def angle_to_target(self, pos, target):
        dx = target[0] - pos.x_val
        dy = target[1] - pos.y_val
        return math.atan2(dy, dx)

    def current_heading(self, velocity):
        return math.atan2(velocity.y_val, velocity.x_val)

    def drive_to_waypoint(self, target_x, target_y, tolerance=1.5, timeout=30):
        start_time = time.time()
        self.controls.throttle = 0.5

        while time.time() - start_time < timeout:
            pos, vel = self.get_position()
            timestamp = time.time()

            desired = self.angle_to_target(pos, (target_x, target_y))
            heading = self.current_heading(vel)
            angle_error = math.atan2(math.sin(desired - heading), math.cos(desired - heading))

            self.controls.steering = max(-1.0, min(1.0, angle_error))
            self.client.setCarControls(self.controls)

            print(f"[{timestamp:.1f}] Moving to ({target_x}, {target_y}) | pos=({pos.x_val:.1f},{pos.y_val:.1f}) | angle error={angle_error:.2f}")
            self.csv_writer.writerow([timestamp, pos.x_val, pos.y_val, pos.z_val])

            distance = math.hypot(target_x - pos.x_val, target_y - pos.y_val)
            if distance < tolerance:
                print(f"Reached waypoint ({target_x}, {target_y})")
                break

            time.sleep(0.5)

        self.stop()

    def navigate_all(self):
        for i, (x, y) in enumerate(self.waypoints):
            print(f"\nNavigating to waypoint {i+1}/{len(self.waypoints)}: ({x}, {y})")
            self.drive_to_waypoint(x, y)

        print("Navigation complete.")
        self.csv_file.close()

    def stop(self):
        self.controls.throttle = 0.0
        self.controls.steering = 0.0
        self.client.setCarControls(self.controls)

if __name__ == "__main__":
    navigator = AirSimWaypointNavigator("waypoints.csv")
    navigator.navigate_all()
