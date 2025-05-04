# Navigation toward multiple targets with live visualization using Matplotlib and obstacle detection

import airsim
import time
import csv
import math
import matplotlib.pyplot as plt
import numpy as np

class AirSimWaypointNavigatorWithObstacleDetection:
    def __init__(self, waypoint_file):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        self.waypoints = self.load_waypoints(waypoint_file)

        self.csv_file = open("navigate_positions.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z"])

        # Initialize matplotlib for live plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("AirSim Navigation - Live Visualization with Obstacle Detection")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_xlim(-10, 60)  # Adjust axis limits as needed
        self.ax.set_ylim(-10, 60)  # Adjust axis limits as needed
        self.ax.grid(True)

        # Plot waypoints as blue dots
        wp_x, wp_y = zip(*self.waypoints)
        self.ax.plot(wp_x, wp_y, 'bo', label="Waypoints")

        # Line for path traversal
        self.path_line, = self.ax.plot([], [], 'g-', label="Path")

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

    def detect_obstacle(self, max_distance=5.0):
        # Use AirSim's front-facing sonar or LiDAR to detect obstacles
        front_sonar = self.client.getDistanceSensorData(sensor_name="Front_Sonar")
        distance = front_sonar.distance
        if distance < max_distance:
            print(f"Obstacle detected within {max_distance} meters!")
            return True
        return False

    def drive_to_waypoint(self, target_x, target_y, tolerance=1.5, timeout=30):
        start_time = time.time()
        self.controls.throttle = 0.5

        path_x = []
        path_y = []

        while time.time() - start_time < timeout:
            pos, vel = self.get_position()
            timestamp = time.time()

            desired = self.angle_to_target(pos, (target_x, target_y))
            heading = self.current_heading(vel)
            angle_error = math.atan2(math.sin(desired - heading), math.cos(desired - heading))

            self.controls.steering = max(-1.0, min(1.0, angle_error))
            self.client.setCarControls(self.controls)

            # Check for obstacles
            if self.detect_obstacle():
                # Reverse if obstacle detected
                self.controls.throttle = -0.5
                self.client.setCarControls(self.controls)
                time.sleep(1)  # Move backward for 1 second
                print("Reversing due to obstacle...")
                
                # Change direction by slightly altering the steering
                self.controls.steering = 0.5  # Change direction
                self.client.setCarControls(self.controls)
                time.sleep(2)  # Turn for a while before resuming forward motion
                print("Changing direction...")

            # Add current position to the path for visualization
            path_x.append(pos.x_val)
            path_y.append(pos.y_val)

            # Update the plot with the new position
            self.path_line.set_data(path_x, path_y)
            self.ax.plot(pos.x_val, pos.y_val, 'ro')  # Plot current position in red
            plt.pause(0.01)

            # Print information
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

        # Show the final plot
        plt.legend()
        plt.show()

    def stop(self):
        self.controls.throttle = 0.0
        self.controls.steering = 0.0
        self.client.setCarControls(self.controls)

if __name__ == "__main__":
    navigator = AirSimWaypointNavigatorWithObstacleDetection("waypoints.csv")
    navigator.navigate_all()
