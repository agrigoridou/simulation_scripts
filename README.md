# simulation_scripts



This repository contains Python scripts for autonomous vehicle simulation using two major platforms:

- **AirSim** (by Microsoft)
- **CARLA** (Car Learning to Act)

The scripts demonstrate basic navigation, exploration, and simple obstacle avoidance behaviors using sensors like LiDAR and depth cameras.

---

## 📁 Contents

### AirSim Scripts
- airsim_explore.py: Drives the car in a straight line and prints vehicle position every second.
- airsim_avoid_obstacle.py: Uses LiDAR sensor to detect nearby obstacles and stops the car if necessary.

### CARLA Scripts
- carla_explore.py: Spawns a vehicle and drives it straight while displaying its position.
- carla_drive_straight.py: Simple straight-driving logic with random vehicle type.
- carla_avoid_obstacle.py: Uses a depth camera to detect nearby objects and automatically brakes.

---

## 🛠 Requirements

Make sure the following dependencies are installed:

- Python 3.7+
- [AirSim](https://github.com/microsoft/AirSim)
- [CARLA Simulator](https://carla.org/)

Required Python packages:

For **AirSim**:
 ```bash
pip install airsim numpy opencv-python
 ```
## How to Run
 
### For AirSim:
 ```bash
pyenv activate airsimNH
cd ~/UnrealEngine4.27/UnrealEngine 
./Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject
 ```

### For AirSimNH:
 ```bash
pyenv activate airsimNH
cd AirSimNH/LinuxNoEditor
./AirSimNH.sh -ResX=640 -ResY=480 -windowed
 ```

### For CARLA:

for server:
 ```bash
    cd path/to/carla/root
    ./CarlaUE4.sh
 ```
for client:
 ```bash
   source ~/.bashrc
   pyenv activate carla-0.9.15
 ```
Then run a script:
 ```bash
   python3 carla_explore_map.py
 ```
