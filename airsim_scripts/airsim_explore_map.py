# Navigation toward a target: Calculating angle difference and heading direction to a target (x, y)

import airsim
import time
import csv
import math

class AirSimNavigateToTarget:
    def __init__(self, target_x, target_y):
        # Connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()
        self.target_x = target_x
        self.target_y = target_y

        # Create CSV file to save positions
        self.csv_file = open("navigate_positions.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z"])

    def get_position(self):
        # Returns current position and velocity of the vehicle
        car_state = self.client.getCarState()
        return car_state.kinematics_estimated.position, car_state.kinematics_estimated.linear_velocity

    def angle_to_target(self, pos, target):
        # Computes angle from current position to the target (in radians)
        dx = target[0] - pos.x_val
        dy = target[1] - pos.y_val
        return math.atan2(dy, dx)

    def current_heading(self, velocity):
        # Computes current heading based on the velocity vector
        return math.atan2(velocity.y_val, velocity.x_val)

    def drive_to_target(self, max_time=20, tolerance=1.0):
        start_time = time.time()
        self.controls.throttle = 0.5  # Set constant throttle

        while time.time() - start_time < max_time:
            pos, vel = self.get_position()
            timestamp = time.time()

            # Calculate desired angle and current heading
            desired = self.angle_to_target(pos, (self.target_x, self.target_y))
            heading = self.current_heading(vel)

            # Normalize angle difference to the range [-π, π]
            angle_error = desired - heading
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            # Set steering based on angle error, clamped between -1 and 1
            self.controls.steering = max(-1.0, min(1.0, angle_error))

            # Send control commands to the vehicle
            self.client.setCarControls(self.controls)

            # Print and log current position
            print(f"[{timestamp:.1f}] Position x={pos.x_val:.1f}, y={pos.y_val:.1f} | Error angle: {angle_error:.2f}")
            self.csv_writer.writerow([timestamp, pos.x_val, pos.y_val, pos.z_val])

            # If vehicle is within tolerance distance to the target, stop
            distance = math.sqrt((self.target_x - pos.x_val) ** 2 + (self.target_y - pos.y_val) ** 2)
            if distance < tolerance:
                print("Reached target!")
                break

            time.sleep(0.5)

        self.stop()

    def stop(self):
        # Stop the vehicle and close the CSV file
        self.controls.throttle = 0.0
        self.controls.steering = 0.0
        self.client.setCarControls(self.controls)
        self.csv_file.close()

if __name__ == "__main__":
    # Define target coordinates
    target_x = 50
    target_y = 0

    nav = AirSimNavigateToTarget(target_x, target_y)
    nav.drive_to_target()
