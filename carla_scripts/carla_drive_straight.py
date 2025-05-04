import carla
import time
import random
import pygame
import numpy as np

class CarlaDriveStraight:
    def __init__(self):
        # Connect to the CARLA simulator
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)  # Set connection timeout
        
        # Get the current world instance
        self.world = self.client.get_world()

        # Select a random vehicle blueprint from the blueprint library
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = random.choice(self.blueprint_library.filter('vehicle'))
        
        # Select a random spawn point from the map
        self.spawn_point = random.choice(self.world.get_map().get_spawn_points())

        # Spawn the vehicle in the simulation
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.spawn_point)

        # Create a camera attached to the vehicle
        self.camera = None
        try:
            self.camera = self.create_camera()
        except Exception as e:
            print(f"Error creating camera: {e}")

    def create_camera(self):
        # Define the camera's relative position to the vehicle
        camera_transform = carla.Transform(carla.Location(x=2.5, z=1.0))  # 2.5 meters in front of the car and 1 meter above
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        
        # Create a pygame window
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        camera.listen(self.process_image)
        
        return camera

    def process_image(self, image):
        # Convert the image to an array and display it using pygame
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        
        # Convert the image into a format that pygame can display
        surface = pygame.surfarray.make_surface(array)
        
        # Draw the image on the screen
        self.screen.blit(surface, (0, 0))
        pygame.display.flip()

    def start(self):
        # Drive forward in a straight line with 50% throttle and 0 steering
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

    def show_vehicle_position(self):
        # Get and print the current position of the vehicle
        location = self.vehicle.get_location()
        print(f"Vehicle position: {location.x}, {location.y}, {location.z}")

    def stop(self):
        # Stop the vehicle by setting throttle and steer to zero
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))

    def close(self):
        # Destroy the vehicle actor and clean up the simulation
        self.vehicle.destroy()
        if self.camera:
            self.camera.destroy()
        pygame.quit()

if __name__ == "__main__":
    # Create a CarlaDriveStraight instance and start the simulation
    demo = CarlaDriveStraight()
    if demo.camera:
        demo.start()

        # Drive for 10 seconds while printing position every second and displaying the camera feed
        for _ in range(10):
            demo.show_vehicle_position()
            time.sleep(1)

        # Stop the vehicle and clean up
        demo.stop()
        demo.close()
    else:
        print("Camera creation failed, exiting.")
