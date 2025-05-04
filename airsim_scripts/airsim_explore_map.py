# airsim_explore_map.py
# Waypoint navigation + live viz + robust avoidance + bounding‑box road constraint

import airsim, time, csv, math, random
import numpy as np
import matplotlib.pyplot as plt

class AirSimNavigatorRoadConstrained:
    def __init__(self, waypoint_file, road_bounds):
        # συνδεση
        self.client = airsim.CarClient();  self.client.confirmConnection();  self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # waypoints
        self.waypoints = []
        with open(waypoint_file) as f:
            for r in csv.DictReader(f):
                self.waypoints.append((float(r['x']), float(r['y'])))

        # road bounding box (min_x, max_x, min_y, max_y)
        self.min_x, self.max_x, self.min_y, self.max_y = road_bounds

        # log
        self.log = open("navigate_positions.csv",'w',newline='')
        self.w = csv.writer(self.log)
        self.w.writerow(["t","x","y","z","state"])

        # plot
        self.fig,self.ax=plt.subplots()
        self.ax.set_xlim(self.min_x-5, self.max_x+5)
        self.ax.set_ylim(self.min_y-5, self.max_y+5)
        self.ax.grid(True)
        # plot bounding box
        bx = [self.min_x, self.max_x, self.max_x, self.min_x, self.min_x]
        by = [self.min_y, self.min_y, self.max_y, self.max_y, self.min_y]
        self.ax.plot(bx, by, 'k--', label="Road bounds")
        # plot waypoints
        wp_x, wp_y = zip(*self.waypoints)
        self.ax.plot(wp_x, wp_y, 'bo', label="Waypoints")
        self.path_line,=self.ax.plot([],[],'g-',label="Path")

    def get_state(self):
        s = self.client.getCarState().kinematics_estimated
        return s.position, s.linear_velocity

    def detect_obstacle_depth(self, thr=5.0, window=20):
        img = self.client.simGetImages([airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])[0]
        depth = airsim.get_pfm_array(img)
        h,w = depth.shape; cy, cx = h//2, w//2; win=window//2
        center = depth[cy-win:cy+win, cx-win:cx+win]
        return float(np.min(center)) < thr

    def enforce_bounds(self, pos):
        # αν έξω από bounds, επιστρέφουμε ένα “ψευδο‑waypoint” στο κέντρο
        if pos.x_val < self.min_x or pos.x_val > self.max_x or pos.y_val < self.min_y or pos.y_val > self.max_y:
            # στόχος = κέντρο δρόμου
            cx = (self.min_x + self.max_x)/2
            cy = (self.min_y + self.max_y)/2
            return True, (cx, cy)
        return False, None

    def drive_to(self, tx, ty):
        t0=time.time(); state="DRIVE"; phase=0; last_pos=None; stuck_t=time.time()
        px,py=[],[]
        while time.time()-t0<30:
            pos,vel = self.get_state(); x,y=pos.x_val,pos.y_val; ts=time.time()
            # bounding‑box check
            offroad, newt = self.enforce_bounds(pos)
            if offroad:
                tx,ty = newt
                state="BOUNDS"
            # state machine
            if state in ("DRIVE","BOUNDS"):
                # steering toward (tx,ty)
                ang = math.atan2(ty-y, tx-x)
                head = math.atan2(vel.y_val, vel.x_val)
                err = math.atan2(math.sin(ang-head), math.cos(ang-head))
                self.controls.steering = max(-1, min(1, err))
                self.controls.throttle = 0.3
                self.client.setCarControls(self.controls)
                # obstacle?
                if self.detect_obstacle_depth(thr=4.0):
                    state="BACKUP"; phase=ts; self.controls.throttle=0; self.client.setCarControls(self.controls)
            elif state=="BACKUP":
                if ts-phase<1.0:
                    self.controls.throttle=-0.3; self.controls.steering=0; self.client.setCarControls(self.controls)
                else:
                    state="TURN"; phase=ts
                    self.controls.throttle=0; self.controls.steering=random.choice([-0.6,0.6])
                    self.client.setCarControls(self.controls)
            elif state=="TURN":
                if ts-phase<1.0:
                    self.client.setCarControls(self.controls)
                else:
                    state="DRIVE"; time.sleep(0.2)
            # log & viz
            px.append(x); py.append(y)
            self.path_line.set_data(px,py)
            self.ax.plot(x,y,'ro'); plt.pause(0.01)
            self.w.writerow([ts,x,y,pos.z_val,state])
            # reached original waypoint?
            if not offroad and math.hypot(tx-x, ty-y)<1.5:
                print(f"Reached ({tx:.1f},{ty:.1f})"); break
            time.sleep(0.1)
        # stop
        self.controls.throttle=0; self.controls.steering=0; self.client.setCarControls(self.controls)

    def run(self):
        for i,(x,y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            self.drive_to(x,y)
        self.log.close(); plt.legend(); plt.show()

if __name__=="__main__":
    # ορίστε bounds που αντιστοιχούν στο “δρόμο” σας
    road_bounds = (0, 50, 0, 10)   # π.χ. x∈[0,50], y∈[0,10]
    nav = AirSimNavigatorRoadConstrained("waypoints.csv", road_bounds)
    nav.run()
