import airsim, time, pprint, numpy

class LidarTest:
    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def execute(self):
        for _ in range(3):
            # κίνηση
            self.car_controls.throttle, self.car_controls.steering = 0.5, 0
            self.client.setCarControls(self.car_controls); time.sleep(3)
            self.car_controls.steering = 1
            self.client.setCarControls(self.car_controls); time.sleep(3)

            for i in range(1,3):
                # προσθήκη ονόματος Lidar1
                lidarData = self.client.getLidarData("Lidar1")
                if len(lidarData.point_cloud) < 3:
                    print("\tNo Lidar points")
                else:
                    points = numpy.array(lidarData.point_cloud, dtype='f4').reshape(-1,3)
                    print(f"\tRead {len(points)} points at ts={lidarData.time_stamp}")
                time.sleep(1)

    def stop(self):
        self.client.reset(); self.client.enableApiControl(False)

if __name__=="__main__":
    test = LidarTest()
    try: test.execute()
    finally: test.stop()
