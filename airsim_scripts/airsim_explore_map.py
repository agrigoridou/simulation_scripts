import airsim
import time
import csv

class AirSimExplore:
    def __init__(self):
        # Connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # Prepare CSV file
        self.csv_file = open("vehicle_positions.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z"])

    def start(self):
        # Drive forward with 50% throttle and no steering
        self.controls.throttle = 0.5
        self.controls.steering = 0.0
        self.client.setCarControls(self.controls)

    def show_vehicle_position(self):
        # Get the current position of the car
        car_state = self.client.getCarState()
        position = car_state.kinematics_estimated.position
        timestamp = time.time()

        # Print the position
        print(f"Time: {timestamp:.2f}, Position: x={position.x_val:.2f}, y={position.y_val:.2f}, z={position.z_val:.2f}")

        # Write to CSV
        self.csv_writer.writerow([timestamp, position.x_val, position.y_val, position.z_val])

    def stop(self):
        # Stop the car
        self.controls.throttle = 0.0
        self.controls.steering = 0.0
        self.client.setCarControls(self.controls)

        # Close CSV file
        self.csv_file.close()

if __name__ == "__main__":
    demo = AirSimExplore()
    demo.start()

    # Drive and log positions for 10 seconds
    for _ in range(10):
        demo.show_vehicle_position()
        time.sleep(1)

    demo.stop()
