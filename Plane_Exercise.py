# Simple plane Simulation Exercise to master before simulating other objects in Pybullet
import pybullet as p  # Import needed pybullet library after installing it with pip in terminal
import pybullet_data

# Connect to Pybullet Servers
Connecting_to_Pybullet = p.connect(p.GUI)  # Connecting to graphical user interface Version of Pybullet

# Setting the Gravity of the simulation
Gravity = p.setGravity(0, 0, -9.8)
# In pybullet the y-axis is front & back | x-axis is left & right | z-axis is up & down

# Setting the coordinates

coordinates = [0, 0, 0]
orientation = [0, 0, 0]

# Coordinates for the simulation

intial_Coordinates = p.getQuaternionFromEuler(orientation)

# Allows us to load in URDF files easily using the p.load function
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf",coordinates, intial_Coordinates)

# Loop to contniue the simulation without closing

while True:
    p.stepSimulation()
    p.setTimeStep(1/240)