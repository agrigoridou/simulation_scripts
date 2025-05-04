import airsim

client = airsim.CarClient()
client.confirmConnection()

state = client.getCarState()
pos = state.kinematics_estimated.position
print(f"Position: ({pos.x_val}, {pos.y_val}, {pos.z_val})")
