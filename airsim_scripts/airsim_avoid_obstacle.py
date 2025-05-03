# airsim_avoid_obstacle.py

import airsim
import numpy as np
import time

class AirSimAvoidObstacle:
    def __init__(self):
        # Connect to AirSim car client
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        
        # Enable API control and arm the vehicle
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        # Define lidar sensor name and obstacle detection flag
        self.lidar_name = 'LidarSensor1'
        self.obstacle_detected = False

    def get_lidar_distance(self):
        # Get lidar data from the specified sensor
        lidar_data = self.client.getLidarData(lidar_name=self.lidar_name)
        
        # Check if lidar has returned any data
        if len(lidar_data.point_cloud) < 3:
            return None

        # Convert point cloud to numpy array and reshape to (N, 3)
        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        
        # Calculate Euclidean distances to all points
        distances = np.linalg.norm(points, axis=1)
        
        # Return the minimum distance (closest obstacle)
        min_distance = np.min(distances)
        return min_distance

    def drive(self):
        print("[INFO] Starting to drive...")
        controls = airsim.CarControls()
        controls.throttle = 0.5  # Set initial forward throttle

        for _ in range(50):  # Loop for ~10 seconds (50 x 0.2s)
            min_dist = self.get_lidar_distance()
            if min_dist and min_dist < 3.0:
                # Obstacle detected within 3 meters, apply brakes
                print(f"[WARNING] Obstacle at {min_dist:.2f}m. Stopping.")
                controls.throttle = 0.0
                controls.brake = 1.0
            else:
                # No obstacle nearby, keep driving
                controls.throttle = 0.5
                controls.brake = 0.0

            # Send updated controls to the car
            self.client.setCarControls(controls)
            time.sleep(0.2)

    def stop(self):
        # Stop the car by releasing throttle and applying brakes
        controls = airsim.CarControls()
        controls.throttle = 0.0
        controls.brake = 1.0
        self.client.setCarControls(controls)
        print("[INFO] Vehicle stopped.")

if __name__ == "__main__":
    try:
        demo = AirSimAvoidObstacle()
        demo.drive()
    finally:
        # Ensure the vehicle stops even if an error occurs
        demo.stop()
