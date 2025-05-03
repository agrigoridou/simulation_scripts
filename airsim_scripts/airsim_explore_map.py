import airsim
import time

class AirSimExplore:
    def __init__(self):
        # Connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()

    def start(self):
        # Start driving forward in a straight line with 50% throttle and no steering
        self.client.setCarControls(airsim.CarControls(throttle=0.5, steer=0.0))

    def show_vehicle_position(self):
        # Get the current state of the car
        car_state = self.client.getCarState()
        position = car_state.position
        
        # Print the current position of the car
        print(f"Vehicle position: {position.x_val}, {position.y_val}, {position.z_val}")

    def stop(self):
        # Stop the car by setting throttle and steering to zero
        self.client.setCarControls(airsim.CarControls(throttle=0.0, steer=0.0))

if __name__ == "__main__":
    # Create an instance of the AirSimExplore class
    demo = AirSimExplore()
    
    # Start the car
    demo.start()

    # Drive and print the car's position every second for 10 seconds
    for _ in range(10):
        demo.show_vehicle_position()
        time.sleep(1)

    # Stop the car
    demo.stop()
