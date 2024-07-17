import pybullet as p
import time
import pybullet_data
from pynput import keyboard

# Connect to PyBullet server
p.connect(p.GUI)

# Set gravity constant for simulation
p.setGravity(0, 0, -9.8)

# Add the additional search path for seamless use of URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set initial coordinates and orientation for the plane
coordinates = [0, 0, 0]
orientation = [0, 0, 0]  # Euler Angles Roll, Pitch, Yaw
initial_orientation = p.getQuaternionFromEuler(orientation)

# Load the plane
planeId = p.loadURDF("plane.urdf", coordinates, initial_orientation)

# Set initial coordinates and orientation for r2d2


# Load the r2d2 model
r2d2Id = p.loadURDF("racecar/racecar.urdf")

# Getting & printing the number of joints of r2d2
num_joints = p.getNumJoints(r2d2Id)
print(num_joints)

# print Information of joints in the R2D2 robot
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(r2d2Id, joint_index)
    print(f"Joints {joint_index}: {joint_info}")
