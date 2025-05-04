import airsim
import time

class AirSimDriveStraight:
    def __init__(self):
        # Connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def start(self):
        # Drive the car straight with 50% throttle and no steering
        controls = airsim.CarControls()
        controls.throttle = 0.5
        controls.steering = 0.0
        self.client.setCarControls(controls)

    def show_vehicle_position(self):
        # Get the current position of the car and print it
        car_state = self.client.getCarState()
        position = car_state.kinematics_estimated.position
        print(f"Vehicle position: x={position.x_val:.2f}, y={position.y_val:.2f}, z={position.z_val:.2f}")

    def stop(self):
        # Stop the car by setting throttle and steering to zero
        controls = airsim.CarControls()
        controls.throttle = 0.0
        controls.steering = 0.0
        self.client.setCarControls(controls)

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
