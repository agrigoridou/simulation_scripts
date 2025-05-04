import time
import numpy as np
import airsim
import matplotlib.pyplot as plt

class RoadNavigator:
    def __init__(self, client, waypoints):
        self.client = client
        self.waypoints = waypoints
        self.ROAD_LABEL = np.array([246, 159, 142])  # RGB color for road (modify this as needed)
        self.COLOR_THRESHOLD = 200  # Relaxed color distance threshold

        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        car_controls = airsim.CarControls()
        car_controls.brake = 0
        self.client.setCarControls(car_controls)

        # Initialize matplotlib for live plotting
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-150, 150)  # Set x range for live plot
        self.ax.set_ylim(-150, 150)  # Set y range for live plot
        self.ax.set_xlabel('X position')
        self.ax.set_ylabel('Y position')

        self.plot_points, = self.ax.plot([], [], 'bo', label="Waypoints")
        self.plot_vehicle, = self.ax.plot([], [], 'go', label="Vehicle Position")

    def update_live_plot(self, vehicle_pos):
        # Update live plot
        self.plot_vehicle.set_data(vehicle_pos[0], vehicle_pos[1])

        # Update waypoints
        waypoints_x = [wp[0] for wp in self.waypoints]
        waypoints_y = [wp[1] for wp in self.waypoints]
        self.plot_points.set_data(waypoints_x, waypoints_y)

        # Display updated plot
        plt.draw()
        plt.pause(0.1)

    def get_segmentation_center_label(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        if responses is None or len(responses) == 0:
            print("Error: No segmentation image received.")
            return -1
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        
        # Get image dimensions from the response
        img_response = responses[0]
        height = img_response.height
        width = img_response.width

        # Reshape the image to the correct dimensions
        img = img1d.reshape((height, width, 3))  # Reshape to 3 channels (RGB)

        # Check the center pixel of the image
        center_pixel = img[height // 2, width // 2]
        
        # Calculate color distance from road color
        color_diff = np.linalg.norm(center_pixel - self.ROAD_LABEL)

        # Return the color difference
        return color_diff

    def drive_to_waypoint(self, x, y, z=-1):
        state = self.client.getCarState()
        start_pos = state.kinematics_estimated.position
        if z == -1:
            z = start_pos.z_val  # Use current altitude

        dx = x - start_pos.x_val
        dy = y - start_pos.y_val
        distance = np.sqrt(dx**2 + dy**2)

        velocity = 5
        duration = distance / velocity

        controls = airsim.CarControls()
        controls.throttle = 0.5
        controls.steering = 0
        self.client.setCarControls(controls)

        start_time = time.time()
        while time.time() - start_time < duration:
            # Update the live plot
            vehicle_pos = (state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val)
            self.update_live_plot(vehicle_pos)

            # Get the segmentation image and check road color
            color_diff = self.get_segmentation_center_label()
            print(f"Color difference: {color_diff}")
            if color_diff > self.COLOR_THRESHOLD:  # If color distance exceeds threshold, consider it off-road
                print("Went off-road!")
                controls = airsim.CarControls()
                controls.throttle = 0.0
                controls.brake = 1.0
                self.client.setCarControls(controls)
                return False
            time.sleep(0.5)

        controls = airsim.CarControls()
        controls.throttle = 0.0
        controls.brake = 1.0
        self.client.setCarControls(controls)
        return True

    def run(self):
        print("Starting navigation...")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            ok = self.drive_to_waypoint(x, y)
            if not ok:
                print("Navigation interrupted due to going off-road.")
                break
        print("Navigation completed or interrupted.")

# Example usage:
client = airsim.CarClient()
client.confirmConnection()

# Example waypoints
waypoints = [
    (128.01309204101562, 34.42572021484375),
    (128.014, 34.426),
    (128.015, 34.427),
    (128.016, 34.428),
    (128.017, 34.429),
    (128.018, 34.430)
]

# Initialize and run the navigation
navigator = RoadNavigator(client, waypoints)
navigator.run()
