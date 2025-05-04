# Navigation toward multiple targets with live visualization using Matplotlib and depth‑based obstacle detection
# Method: Waypoint-based navigation with obstacle avoidance via depth‑camera

import airsim
import time
import csv
import math
import matplotlib.pyplot as plt
import numpy as np

class AirSimWaypointNavigatorWithDepthAvoidance:
    def __init__(self, waypoint_file):
        # connect to AirSim CarClient
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # load waypoints
        self.waypoints = self.load_waypoints(waypoint_file)

        # prepare CSV log
        self.csv_file = open("navigate_positions.csv", mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "z"])

        # set up live plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("AirSim Navigation – Depth‑Based Obstacle Avoidance")
        self.ax.set_xlabel("X Position");  self.ax.set_ylabel("Y Position")
        self.ax.set_xlim(-10, 60);  self.ax.set_ylim(-10, 60)
        self.ax.grid(True)
        # plot waypoints
        wp_x, wp_y = zip(*self.waypoints)
        self.ax.plot(wp_x, wp_y, 'bo', label="Waypoints")
        self.path_line, = self.ax.plot([], [], 'g-', label="Path")

    def load_waypoints(self, filename):
        waypoints = []
        with open(filename) as f:
            for row in csv.DictReader(f):
                waypoints.append((float(row['x']), float(row['y'])))
        return waypoints

    def get_position(self):
        state = self.client.getCarState()
        return state.kinematics_estimated.position, state.kinematics_estimated.linear_velocity

    def angle_to_target(self, pos, target):
        return math.atan2(target[1] - pos.y_val, target[0] - pos.x_val)

    def current_heading(self, vel):
        return math.atan2(vel.y_val, vel.x_val)

    def detect_obstacle_depth(self, threshold=10.0):
        # retrieve a depth image (camera "1" is default depth camera)
        responses = self.client.simGetImages([
            airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)
        ])
        if not responses: 
            return False
        depth = airsim.get_pfm_array(responses[0])
        # find minimum distance in front
        min_dist = float(np.min(depth))
        if min_dist < threshold:
            print(f"Obstacle at ~{min_dist:.1f}m ahead")
            return True
        return False

    def drive_to_waypoint(self, tx, ty, tolerance=1.5, timeout=30):
        start = time.time()
        self.controls.throttle = 0.4   # moderate speed
        path_x, path_y = [], []

        while time.time() - start < timeout:
            pos, vel = self.get_position()
            ts = time.time()

            # compute steering
            desired = self.angle_to_target(pos, (tx, ty))
            heading = self.current_heading(vel)
            err = math.atan2(math.sin(desired - heading), math.cos(desired - heading))
            self.controls.steering = max(-1, min(1, err))
            self.client.setCarControls(self.controls)

            # obstacle avoidance
            if self.detect_obstacle_depth(threshold=8.0):
                # reverse
                self.controls.throttle = -0.3
                self.client.setCarControls(self.controls)
                time.sleep(1)
                # turn away
                self.controls.steering = 0.7
                self.controls.throttle = 0.4
                self.client.setCarControls(self.controls)
                time.sleep(1)
                print("Avoided obstacle, resuming")

            # log & plot
            path_x.append(pos.x_val);  path_y.append(pos.y_val)
            self.path_line.set_data(path_x, path_y)
            self.ax.plot(pos.x_val, pos.y_val, 'ro')
            plt.pause(0.01)

            self.csv_writer.writerow([ts, pos.x_val, pos.y_val, pos.z_val])
            print(f"[{ts:.1f}] to ({tx},{ty}) pos=({pos.x_val:.1f},{pos.y_val:.1f}) err={err:.2f}")

            if math.hypot(tx - pos.x_val, ty - pos.y_val) < tolerance:
                print(f"Reached ({tx},{ty})")
                break

            time.sleep(0.3)

        self.stop()

    def navigate_all(self):
        for i,(x,y) in enumerate(self.waypoints):
            print(f"\n→ Waypoint {i+1}/{len(self.waypoints)}: ({x},{y})")
            self.drive_to_waypoint(x, y)
        self.csv_file.close()
        plt.legend();  plt.show()

    def stop(self):
        self.controls.throttle = 0;  self.controls.steering = 0
        self.client.setCarControls(self.controls)

if __name__ == "__main__":
    nav = AirSimWaypointNavigatorWithDepthAvoidance("waypoints.csv")
    nav.navigate_all()


////////////////////////////////////////////////////


python3 airsim_explore_map.py
Connected!
Client Ver:1 (Min Req: 1), Server Ver:1 (Min Req: 1)


→ Waypoint 1/4: (50.0,0.0)
Obstacle at ~1.7m ahead
Avoided obstacle, resuming
[1746365225.6] to (50.0,0.0) pos=(-0.0,-0.0) err=0.00
Obstacle at ~1.8m ahead
Avoided obstacle, resuming
[1746365228.1] to (50.0,0.0) pos=(1.8,0.8) err=-0.62
Obstacle at ~1.6m ahead
Avoided obstacle, resuming
[1746365230.5] to (50.0,0.0) pos=(6.8,2.8) err=-0.86
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365233.0] to (50.0,0.0) pos=(11.3,4.9) err=-0.42
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365235.4] to (50.0,0.0) pos=(11.3,4.9) err=-0.13
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365237.7] to (50.0,0.0) pos=(11.3,4.9) err=0.56
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365240.1] to (50.0,0.0) pos=(11.3,4.9) err=-2.67
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365242.5] to (50.0,0.0) pos=(11.3,4.9) err=0.43
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365244.9] to (50.0,0.0) pos=(11.3,4.9) err=-2.70
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365247.3] to (50.0,0.0) pos=(11.3,4.9) err=-2.63
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365249.7] to (50.0,0.0) pos=(11.3,4.9) err=-2.60
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365252.1] to (50.0,0.0) pos=(11.3,4.9) err=-2.76
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365254.5] to (50.0,0.0) pos=(11.3,4.9) err=-2.74

→ Waypoint 2/4: (50.0,50.0)
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365256.8] to (50.0,50.0) pos=(11.3,4.9) err=0.86
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365259.2] to (50.0,50.0) pos=(11.3,4.9) err=1.44
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365261.6] to (50.0,50.0) pos=(11.3,4.9) err=-1.75
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365264.0] to (50.0,50.0) pos=(11.3,4.9) err=-1.84
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365266.4] to (50.0,50.0) pos=(11.3,4.9) err=-1.60
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365268.8] to (50.0,50.0) pos=(11.3,4.9) err=1.29
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365271.2] to (50.0,50.0) pos=(11.3,4.9) err=1.62
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365273.6] to (50.0,50.0) pos=(11.3,4.9) err=1.37
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365276.0] to (50.0,50.0) pos=(11.3,4.9) err=1.46
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365278.4] to (50.0,50.0) pos=(11.3,4.9) err=-1.76
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365280.8] to (50.0,50.0) pos=(11.3,4.9) err=1.50
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365283.1] to (50.0,50.0) pos=(11.3,4.9) err=-1.75
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365285.5] to (50.0,50.0) pos=(11.3,4.9) err=1.13

→ Waypoint 3/4: (0.0,50.0)
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365287.9] to (0.0,50.0) pos=(11.3,4.9) err=2.47
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365290.3] to (0.0,50.0) pos=(11.3,4.9) err=-0.74
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365292.7] to (0.0,50.0) pos=(11.3,4.9) err=-0.88
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365295.1] to (0.0,50.0) pos=(11.3,4.9) err=2.11
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365297.5] to (0.0,50.0) pos=(11.3,4.9) err=-0.90
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365299.9] to (0.0,50.0) pos=(11.3,4.9) err=2.45
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365302.3] to (0.0,50.0) pos=(11.3,4.9) err=2.26
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365304.7] to (0.0,50.0) pos=(11.3,4.9) err=-0.77
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365307.1] to (0.0,50.0) pos=(11.3,4.9) err=-0.84
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365309.5] to (0.0,50.0) pos=(11.3,4.9) err=1.86
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365311.8] to (0.0,50.0) pos=(11.3,4.9) err=2.28
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365314.2] to (0.0,50.0) pos=(11.3,4.9) err=2.12
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365316.6] to (0.0,50.0) pos=(11.3,5.0) err=2.24

→ Waypoint 4/4: (0.0,0.0)
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365319.0] to (0.0,0.0) pos=(11.3,5.0) err=-2.17
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365321.4] to (0.0,0.0) pos=(11.3,5.0) err=0.96
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365323.8] to (0.0,0.0) pos=(11.3,5.0) err=1.08
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365326.2] to (0.0,0.0) pos=(11.3,5.0) err=-2.16
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365328.6] to (0.0,0.0) pos=(11.3,5.0) err=-2.06
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365331.0] to (0.0,0.0) pos=(11.3,5.0) err=-2.11
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365333.4] to (0.0,0.0) pos=(11.3,5.0) err=-2.13
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365335.8] to (0.0,0.0) pos=(11.3,4.9) err=-2.22
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365338.2] to (0.0,0.0) pos=(11.3,4.9) err=-2.30
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365340.6] to (0.0,0.0) pos=(11.3,4.9) err=0.94
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365343.0] to (0.0,0.0) pos=(11.3,4.9) err=-2.32
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365345.4] to (0.0,0.0) pos=(11.3,4.9) err=0.90
Obstacle at ~0.4m ahead
Avoided obstacle, resuming
[1746365347.8] to (0.0,0.0) pos=(11.3,4.9) err=-2.20
(airsimNH) sysadm@icsd20048vm:~/Desktop/simulation_scripts-main/airsim_scripts$  
