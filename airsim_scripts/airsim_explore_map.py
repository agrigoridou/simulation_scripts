# Navigation toward multiple targets with live visualization and robust obstacle avoidance
# Method: Waypoint-based navigation + central‐window depth filtering + stuck detection

import airsim, time, csv, math, random
import numpy as np
import matplotlib.pyplot as plt

class AirSimWaypointNavigatorRobust:
    def __init__(self, waypoint_file):
        # σύνδεση
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # waypoints
        self.waypoints = []
        with open(waypoint_file) as f:
            for r in csv.DictReader(f):
                self.waypoints.append((float(r['x']), float(r['y'])))

        # log
        self.log = open("navigate_positions.csv",'w',newline='')
        self.w = csv.writer(self.log)
        self.w.writerow(["t","x","y","z","state"])

        # plot
        self.fig,self.ax=plt.subplots()
        self.ax.set_xlim(-10,60); self.ax.set_ylim(-10,60); self.ax.grid(True)
        wp_x,wp_y=zip(*self.waypoints)
        self.ax.plot(wp_x,wp_y,'bo',label="WP")
        self.path_line,=self.ax.plot([],[], 'g-')

    def get_state(self):
        s=self.client.getCarState().kinematics_estimated
        return s.position, s.linear_velocity

    def detect_obstacle(self, threshold=5.0, window=20):
        # depth image
        resp = self.client.simGetImages([airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])[0]
        depth = airsim.get_pfm_array(resp)
        h,w = depth.shape
        # κεντρικό παράθυρο
        cy, cx = h//2, w//2
        win = window//2
        center = depth[cy-win:cy+win, cx-win:cx+win]
        mind = np.min(center)
        return mind < threshold

    def navigate_to(self, tx,ty):
        t0=time.time()
        state="DRIVE"; phase=0; last_pos=None; stuck_timer=time.time()
        path_x,path_y = [],[]
        while time.time()-t0<30:
            pos,vel=self.get_state()
            x,y=pos.x_val,pos.y_val
            # stuck detection
            if last_pos:
                if math.hypot(x-last_pos[0], y-last_pos[1])<0.1:
                    if time.time()-stuck_timer>3:
                        # force a big turn
                        self.controls.steering = random.choice([-1,1])
                        self.controls.throttle = 0.3
                        self.client.setCarControls(self.controls)
                        time.sleep(1)
                        state="DRIVE"
                        stuck_timer=time.time()
                else:
                    stuck_timer=time.time()
            last_pos=(x,y)

            # state machine
            if state=="DRIVE":
                # compute steering toward target
                ang = math.atan2(ty-y, tx-x)
                head=math.atan2(vel.y_val, vel.x_val)
                err=math.atan2(math.sin(ang-head), math.cos(ang-head))
                self.controls.steering = max(-1, min(1, err))
                self.controls.throttle = 0.4
                self.client.setCarControls(self.controls)
                # obstacle?
                if self.detect_obstacle():
                    state="BACKUP"; phase=time.time()
                    self.controls.throttle=0; self.client.setCarControls(self.controls)

            elif state=="BACKUP":
                # όπισθεν για 1s
                if time.time()-phase<1.0:
                    self.controls.throttle=-0.3; self.controls.steering=0
                    self.client.setCarControls(self.controls)
                else:
                    state="TURN"; phase=time.time()
                    self.controls.throttle=0; self.controls.steering=random.choice([-0.7,0.7])
                    self.client.setCarControls(self.controls)

            elif state=="TURN":
                # turn in place 1s
                if time.time()-phase<1.0:
                    self.client.setCarControls(self.controls)
                else:
                    state="DRIVE"; time.sleep(0.2)

            # log & plot
            path_x.append(x); path_y.append(y)
            self.path_line.set_data(path_x,path_y)
            self.ax.plot(x,y,'ro'); plt.pause(0.01)
            self.w.writerow([time.time(), x,y,pos.z_val, state])

            # reached?
            if math.hypot(tx-x, ty-y)<1.5:
                print(f"Reached ({tx},{ty})"); break

            time.sleep(0.1)

        # stop
        self.controls.throttle=0; self.controls.steering=0
        self.client.setCarControls(self.controls)

    def run(self):
        for i,(x,y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            self.navigate_to(x,y)
        self.log.close()
        plt.legend(); plt.show()

if __name__=="__main__":
    nav=AirSimWaypointNavigatorRobust("waypoints.csv")
    nav.run()
