import gymnasium as gym
import numpy as np
import carla
import time
import random

class CarlaEnv(gym.Env):
    def __init__(self, host='localhost', port=2000, timeout=10.0, max_sensor_wait=2.0):
        super().__init__()
        self.host = host
        self.port = port
        self.timeout = timeout
        self.max_sensor_wait = max_sensor_wait

        # Connect to CARLA
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()

        # Define observation and action spaces
        self.observation_space = gym.spaces.Dict({
            "image": gym.spaces.Box(low=0, high=255, shape=(5, 128, 128), dtype=np.uint8),
            "state": gym.spaces.Box(
                low=np.array([0.0, -1.0, 0.0, 0.0, 0.0, -np.pi]),
                high=np.array([100.0, 1.0, 1.0, 1.0, 100.0, np.pi]),
                shape=(6,),
                dtype=np.float32
            )
        })
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, 0.0, 0.0]),
            high=np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )

        # Sensor data holders
        self.rgb_image = None
        self.seg_image = None
        self.collision = False
        self.lane_invasion = False

        # Actor/sensor handles
        self.vehicle = None
        self.camera_sensor = None
        self.seg_sensor = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None

    def _cleanup(self):
        # Stop and destroy all sensors
        sensors = [self.camera_sensor, self.seg_sensor, self.collision_sensor, self.lane_invasion_sensor]
        for sensor in sensors:
            if sensor is not None:
                try:
                    sensor.stop()
                    sensor.destroy()
                except Exception:
                    pass
        self.camera_sensor = None
        self.seg_sensor = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None

        # Destroy vehicle
        if self.vehicle is not None:
            try:
                self.vehicle.destroy()
            except Exception:
                pass
            self.vehicle = None

        # Clear sensor data
        self.rgb_image = None
        self.seg_image = None
        self.collision = False
        self.lane_invasion = False

    def reset(self, seed=None, options=None):
        self._cleanup()
        # Optionally reload world for full cleanup (uncomment if needed)
        # self.world = self.client.reload_world(False)
        # self.blueprint_library = self.world.get_blueprint_library()

        # Spawn vehicle
        vehicle_bp = random.choice(self.blueprint_library.filter('vehicle.*'))
        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
        if self.vehicle is None:
            raise RuntimeError("Failed to spawn vehicle.")

        # Attach sensors
        cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '128')
        cam_bp.set_attribute('image_size_y', '128')
        cam_bp.set_attribute('fov', '90')
        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera_sensor = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.vehicle)
        self.camera_sensor.listen(self._on_rgb_image)

        seg_bp = self.blueprint_library.find('sensor.camera.semantic_segmentation')
        seg_bp.set_attribute('image_size_x', '128')
        seg_bp.set_attribute('image_size_y', '128')
        seg_bp.set_attribute('fov', '90')
        self.seg_sensor = self.world.spawn_actor(seg_bp, cam_transform, attach_to=self.vehicle)
        self.seg_sensor.listen(self._on_seg_image)

        collision_bp = self.blueprint_library.find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(), attach_to=self.vehicle)
        self.collision_sensor.listen(self._on_collision)

        lane_bp = self.blueprint_library.find('sensor.other.lane_invasion')
        self.lane_invasion_sensor = self.world.spawn_actor(lane_bp, carla.Transform(), attach_to=self.vehicle)
        self.lane_invasion_sensor.listen(self._on_lane_invasion)

        # Wait for first sensor data with timeout
        if not self._wait_for_sensor_data():
            print("Sensor data not received after reset, cleaning up and retrying...")
            self._cleanup()
            raise RuntimeError("Sensor data not received after reset.")

        # Return initial observation
        return self.get_observation(), {}

    def _wait_for_sensor_data(self):
        start_time = time.time()
        while True:
            if self.rgb_image is not None and self.seg_image is not None:
                return True
            if time.time() - start_time > self.max_sensor_wait:
                return False
            time.sleep(0.01)

    def _on_rgb_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        self.rgb_image = array

    def _on_seg_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        self.seg_image = array

    def _on_collision(self, event):
        self.collision = True

    def _on_lane_invasion(self, event):
        self.lane_invasion = True

    def get_observation(self):
        # Wait for sensor data with timeout
        if not self._wait_for_sensor_data():
            print("Sensor data not received in get_observation, cleaning up and retrying...")
            self._cleanup()
            raise RuntimeError("Sensor data not received in get_observation.")

        # Preprocess images and state
        rgb = cv2.resize(self.rgb_image, (128, 128))
        seg = cv2.resize(self.seg_image, (128, 128))

        # Create masks (example, adjust as needed)
        lane_mask = np.all(seg == [0, 255, 0], axis=2).astype(np.uint8)
        obs_mask = np.all(seg == [255, 0, 0], axis=2).astype(np.uint8)

        image_obs = np.stack([rgb[..., 0], rgb[..., 1], rgb[..., 2], lane_mask, obs_mask], axis=0)

        # State vector (example, adjust as needed)
        velocity = self.vehicle.get_velocity()
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        control = self.vehicle.get_control()
        state_obs = np.array([
            speed,
            control.steer,
            control.throttle,
            control.brake,
            0.0,  # placeholder for waypoint distance
            0.0   # placeholder for waypoint angle
        ], dtype=np.float32)

        return {"image": image_obs, "state": state_obs}

    def step(self, action):
        # Apply action
        steer = float(np.clip(action[0], -1.0, 1.0))
        throttle = float(np.clip(action[1], 0.0, 1.0))
        brake = float(np.clip(action[2], 0.0, 1.0))
        self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

        # Advance simulation
        self.world.tick()

        # Get observation
        obs = self.get_observation()

        # Compute reward (example)
        reward = 1.0 - float(self.collision)

        # Termination condition (example)
        done = self.collision

        info = {}

        return obs, reward, done, False, info

    def close(self):
        self._cleanup()
