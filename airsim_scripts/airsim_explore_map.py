# airsim_explore_map.py
# Waypoint navigation + live viz + robust obstacle avoidance + hard road‑bounds enforcement

import airsim, time, csv, math, random
import numpy as np
import matplotlib.pyplot as plt

class AirSimNavigatorHardBounds:
    def __init__(self, waypoint_file, road_bounds):
        # connect
        self.client = airsim.CarClient(); self.client.confirmConnection(); self.client.enableApiControl(True)
        self.controls = airsim.CarControls()

        # load waypoints
        self.waypoints = []
        with open(waypoint_file) as f:
            for r in csv.DictReader(f):
                self.waypoints.append((float(r['x']), float(r['y'])))

        # road bounds
        self.min_x, self.max_x, self.min_y, self.max_y = road_bounds

        # log & plot
        self.log = open("navigate_positions.csv",'w',newline='')
        self.writer = csv.writer(self.log)
        self.writer.writerow(["t","x","y","z","state"])
        self.fig,self.ax=plt.subplots()
        self.ax.set_xlim(self.min_x-5,self.max_x+5); self.ax.set_ylim(self.min_y-5,self.max_y+5); self.ax.grid(True)
        # draw bounds
        bx=[self.min_x,self.max_x,self.max_x,self.min_x,self.min_x]
        by=[self.min_y,self.min_y,self.max_y,self.max_y,self.min_y]
        self.ax.plot(bx,by,'k--',label="Road bounds")
        # draw waypoints
        wp_x,wp_y=zip(*self.waypoints)
        self.ax.plot(wp_x,wp_y,'bo',label="Waypoints")
        self.path_line,=self.ax.plot([],[],'g-',label="Path")

    def get_state(self):
        s=self.client.getCarState().kinematics_estimated
        return s.position, s.linear_velocity

    def detect_obstacle(self, thr=4.0, win=20):
        img=self.client.simGetImages([airsim.ImageRequest(1,airsim.ImageType.DepthPlanar,True)])[0]
        depth=airsim.get_pfm_array(img)
        h,w=depth.shape; cy, cx=h//2, w//2; w2=win//2
        center=depth[cy-w2:cy+w2, cx-w2:cx+w2]
        return float(np.min(center))<thr

    def enforce_bounds(self, pos):
        # αν βγήκε έξω, επιστρέφουμε True
        return not(self.min_x <= pos.x_val <= self.max_x and self.min_y <= pos.y_val <= self.max_y)

    def drive_to(self, tx, ty):
        t0=time.time(); state="DRIVE"; phase=0
        path_x,path_y=[],[]
        while time.time()-t0<40:
            pos,vel=self.get_state(); x,y=pos.x_val,pos.y_val; ts=time.time()
            offroad=self.enforce_bounds(pos)

            # αν εκτός bounds, επιβάλλουμε HARD επιστροφή
            if offroad:
                state="RECOVER"
                # φρένο + όπισθεν + στροφή προς κέντρο δρόμου
                cx=(self.min_x+self.max_x)/2; cy=(self.min_y+self.max_y)/2
                ang=math.atan2(cy-y, cx-x)
                # reverse + steer toward center
                self.controls.throttle=-0.3
                self.controls.steering=max(-1, min(1, ang))
                self.client.setCarControls(self.controls)
                print(f"[{ts:.1f}] OFFROAD → recovering toward center")
            else:
                # κανονική state machine
                if state=="DRIVE":
                    # steer to target
                    ang=math.atan2(ty-y, tx-x)
                    head=math.atan2(vel.y_val, vel.x_val)
                    err=math.atan2(math.sin(ang-head), math.cos(ang-head))
                    self.controls.steering=max(-1,min(1,err)); self.controls.throttle=0.3
                    self.client.setCarControls(self.controls)
                    if self.detect_obstacle():
                        state="BACK"; phase=ts; self.controls.throttle=0; self.client.setCarControls(self.controls)
                elif state=="BACK":
                    if ts-phase<1.0:
                        self.controls.throttle=-0.3; self.controls.steering=0; self.client.setCarControls(self.controls)
                    else:
                        state="TURN"; phase=ts
                        self.controls.throttle=0; self.controls.steering=random.choice([-0.7,0.7]); self.client.setCarControls(self.controls)
                elif state=="TURN":
                    if ts-phase<1.0:
                        self.client.setCarControls(self.controls)
                    else:
                        state="DRIVE"; time.sleep(0.2)

            # log & viz
            path_x.append(x); path_y.append(y)
            self.path_line.set_data(path_x,path_y)
            self.ax.plot(x,y,'ro'); plt.pause(0.01)
            self.writer.writerow([ts,x,y,pos.z_val,state])

            # check arrival (μόνο αν όχι offroad)
            if not offroad and math.hypot(tx-x,ty-y)<1.5:
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
    # ρύθμισε ώστε να ταιριάζει στο "δρόμο" σου
    bounds = (0,50,0,10)
    nav=AirSimNavigatorHardBounds("waypoints.csv", bounds)
    nav.run()
