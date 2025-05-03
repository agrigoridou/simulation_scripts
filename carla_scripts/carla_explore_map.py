import carla
import time
import random

class CarlaExplore:
    def __init__(self):
        # Connect to the CARLA simulator
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Create a vehicle actor
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle')[0]  # Select a generic vehicle
        self.spawn_point = random.choice(self.world.get_map().get_spawn_points())  # Choose a random spawn point
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.spawn_point)  # Spawn the vehicle

    def start(self):
        # Move the vehicle forward in a straight line
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

    def show_vehicle_position(self):
        # Print the current location of the vehicle
        location = self.vehicle.get_location()
        print(f"Vehicle position: {location.x}, {location.y}, {location.z}")

    def stop(self):
        # Stop the vehicle
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))

    def close(self):
        # Destroy the vehicle actor and clean up
        self.vehicle.destroy()

if __name__ == "__main__":
    demo = CarlaExplore()
    demo.start()

    # Drive for 10 seconds while printing position each second
    for _ in range(10):
        demo.show_vehicle_position()
        time.sleep(1)

    demo.stop()
    demo.close()
