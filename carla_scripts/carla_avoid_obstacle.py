# carla_avoid_obstacle.py

import carla
import time
import numpy as np
import random
import cv2

class CarlaAvoidObstacle:
    def __init__(self):
        # Connect to CARLA simulator
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Select a specific vehicle blueprint (Tesla Model 3)
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]

        # Choose a random spawn point and spawn the vehicle
        self.spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.spawn_point)

        # Setup a depth camera sensor
        self.camera_bp = self.blueprint_library.find('sensor.camera.depth')
        self.camera_bp.set_attribute("image_size_x", "320")
        self.camera_bp.set_attribute("image_size_y", "240")
        self.camera_bp.set_attribute("fov", "90")

        # Attach the camera to the front of the vehicle
        self.camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(self.camera_bp, self.camera_transform, attach_to=self.vehicle)

        # Set up a callback to process depth images
        self.camera.listen(lambda image: self.process_depth(image))

        self.obstacle_detected = False

    def process_depth(self, image):
        # Convert the image to logarithmic depth
        image.convert(carla.ColorConverter.LogarithmicDepth)

        # Convert image data to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))

        # Extract grayscale values from the red channel
        gray = array[:, :, 0]

        # Focus on the center region of the depth image
        center = gray[100:140, 140:180]
        mean_depth = np.mean(center)

        # If the mean depth is low, an obstacle is close
        if mean_depth < 30:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def drive(self):
        print("[INFO] Starting to drive...")
        for _ in range(50):  # Drive for approximately 10 seconds
            if self.obstacle_detected:
                print("[WARNING] Obstacle detected! Applying brakes.")
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            else:
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
            time.sleep(0.2)

    def cleanup(self):
        # Stop the camera and destroy both camera and vehicle actors
        self.camera.stop()
        self.camera.destroy()
        self.vehicle.destroy()
        print("[INFO] Shutdown and cleanup complete.")

if __name__ == "__main__":
    try:
        demo = CarlaAvoidObstacle()
        demo.drive()
    finally:
        demo.cleanup()
