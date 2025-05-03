import carla
import time
import random

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

if __name__ == "__main__":
    # Create a CarlaDriveStraight instance and start the simulation
    demo = CarlaDriveStraight()
    demo.start()

    # Drive for 10 seconds while printing position every second
    for _ in range(10):
        demo.show_vehicle_position()
        time.sleep(1)

    # Stop the vehicle and clean up
    demo.stop()
    demo.close()
