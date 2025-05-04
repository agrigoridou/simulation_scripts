class RoadNavigator:
    def __init__(self, client, waypoints):
        self.client = client
        self.waypoints = waypoints
        self.ROAD_LABEL = np.array([246, 159, 142])  # Ο RGB χρωματικός κωδικός του δρόμου
        self.COLOR_THRESHOLD = 200  # Χαλαρότερο όριο για την απόσταση χρώματος

        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        car_controls = airsim.CarControls()
        car_controls.brake = 0
        self.client.setCarControls(car_controls)

        # Ορίζουμε το matplotlib για να εμφανίζουμε τα live plots
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-150, 150)  # Περιοχή x για την προβολή του live plot
        self.ax.set_ylim(-150, 150)  # Περιοχή y για την προβολή του live plot
        self.ax.set_xlabel('X position')
        self.ax.set_ylabel('Y position')

        self.plot_points, = self.ax.plot([], [], 'bo', label="Waypoints")
        self.plot_vehicle, = self.ax.plot([], [], 'go', label="Vehicle Position")

    def update_live_plot(self, vehicle_pos):
        # Ενημέρωση του live plot
        self.plot_vehicle.set_data(vehicle_pos[0], vehicle_pos[1])

        # Ενημέρωση των waypoints
        waypoints_x = [wp[0] for wp in self.waypoints]
        waypoints_y = [wp[1] for wp in self.waypoints]
        self.plot_points.set_data(waypoints_x, waypoints_y)

        # Εμφάνιση του ενημερωμένου plot
        plt.draw()
        plt.pause(0.1)

    def get_segmentation_center_label(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])
        if responses is None or len(responses) == 0:
            print("Σφάλμα: δεν ελήφθη εικόνα από το segmentation.")
            return -1
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        
        # Λήψη διαστάσεων εικόνας από την απάντηση
        img_response = responses[0]
        height = img_response.height
        width = img_response.width

        # Αναμορφώνουμε την εικόνα με τις σωστές διαστάσεις
        img = img1d.reshape((height, width, 3))  # Προσαρμόζουμε τις διαστάσεις της εικόνας

        # Ελέγχουμε το κέντρο της εικόνας
        center_pixel = img[height // 2, width // 2]
        
        # Υπολογίζουμε την απόσταση του χρώματος από το χρώμα του δρόμου
        color_diff = np.linalg.norm(center_pixel - self.ROAD_LABEL)

        # Επιστρέφουμε την απόσταση του χρώματος
        return color_diff

    def drive_to_waypoint(self, x, y, z=-1):
        state = self.client.getCarState()
        start_pos = state.kinematics_estimated.position
        if z == -1:
            z = start_pos.z_val  # Χρησιμοποιούμε το τρέχον ύψος

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
            # Ενημέρωση του live plot
            vehicle_pos = (state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val)
            self.update_live_plot(vehicle_pos)

            # Λήψη της εικόνας για τον έλεγχο του δρόμου
            color_diff = self.get_segmentation_center_label()
            print(f"Color difference: {color_diff}")
            if color_diff > self.COLOR_THRESHOLD:  # Αν η απόσταση χρώματος είναι μεγαλύτερη από το χαλαρωμένο όριο
                print("Βγήκαμε εκτός δρόμου!")
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
        print("Ξεκινάμε πλοήγηση...")
        for i, (x, y) in enumerate(self.waypoints):
            print(f"→ WP {i+1}/{len(self.waypoints)}: ({x},{y})")
            ok = self.drive_to_waypoint(x, y)
            if not ok:
                print("Πλοήγηση διακόπηκε λόγω εξόδου από δρόμο.")
                break
        print("Πλοήγηση ολοκληρώθηκε ή διεκόπη.")
