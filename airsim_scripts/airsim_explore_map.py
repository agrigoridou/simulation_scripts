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


////////////////////////////////////////////////////

python3 airsim_explore_map.py
Connected!
Client Ver:1 (Min Req: 1), Server Ver:1 (Min Req: 1)

[1746363045.6] Position x=-0.0, y=0.0 | Error angle: -0.00
[1746363046.1] Position x=-0.0, y=0.0 | Error angle: -0.42
[1746363046.6] Position x=0.1, y=-0.0 | Error angle: 0.16
[1746363047.1] Position x=0.9, y=-0.0 | Error angle: -0.05
[1746363047.6] Position x=2.0, y=-0.0 | Error angle: 0.03
[1746363048.1] Position x=3.3, y=-0.0 | Error angle: -0.00
[1746363048.6] Position x=4.9, y=-0.0 | Error angle: 0.01
[1746363049.1] Position x=6.6, y=-0.0 | Error angle: 0.00
[1746363049.6] Position x=8.6, y=-0.0 | Error angle: 0.00
[1746363050.1] Position x=10.6, y=-0.0 | Error angle: 0.00
[1746363050.6] Position x=12.8, y=-0.0 | Error angle: 0.00
[1746363051.1] Position x=15.0, y=-0.0 | Error angle: 0.00
[1746363051.6] Position x=17.3, y=-0.0 | Error angle: 0.00
[1746363052.1] Position x=19.7, y=-0.0 | Error angle: 0.00
[1746363052.6] Position x=22.1, y=-0.0 | Error angle: 0.00
[1746363053.1] Position x=24.5, y=-0.0 | Error angle: 0.00
[1746363053.6] Position x=27.0, y=-0.0 | Error angle: 0.00
[1746363054.1] Position x=29.5, y=-0.0 | Error angle: 0.00
[1746363054.6] Position x=32.1, y=-0.0 | Error angle: 0.00
[1746363055.1] Position x=34.7, y=-0.0 | Error angle: 0.00
[1746363055.6] Position x=37.2, y=-0.0 | Error angle: -0.00
[1746363056.1] Position x=39.8, y=-0.0 | Error angle: -0.00
[1746363056.6] Position x=42.4, y=-0.0 | Error angle: -0.00
[1746363057.1] Position x=45.0, y=-0.0 | Error angle: -0.00
[1746363057.6] Position x=47.6, y=-0.0 | Error angle: -0.00
[1746363058.1] Position x=50.2, y=0.0 | Error angle: -3.14
Reached target!
