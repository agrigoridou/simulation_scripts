import airsim
import time

class AirSimDriveStraight:
    def __init__(self):
        # Connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()

    def start(self):
        # Drive the car straight with 50% throttle and no steering
        self.client.setCarControls(airsim.CarControls(throttle=0.5, steering=0.0))

    def show_vehicle_position(self):
        # Get the current position of the car and print it
        car_state = self.client.getCarState()
        position = car_state.position
        print(f"Vehicle position: {position.x_val}, {position.y_val}, {position.z_val}")

    def stop(self):
        # Stop the car by setting throttle and steer to zero
        self.client.setCarControls(airsim.CarControls(throttle=0.0, steering=0.0))

if __name__ == "__main__":
    # Create an instance of the AirSimDriveStraight class
    demo = AirSimDriveStraight()
    
    # Start driving straight
    demo.start()

    # Drive and print position for 10 seconds
    for _ in range(10):
        demo.show_vehicle_position()
        time.sleep(1)

    # Stop the car
    demo.stop()
