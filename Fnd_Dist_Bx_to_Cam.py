import pybullet as p
import time
import pybullet_data
import numpy as np
import threading

# We want to connect to the pybullet servers

Connect_to_Servers = p.connect(p.GUI)

# Setting the gravity of the simulation

Gravity = p.setGravity(0, 0, -9.8)

# Allows me to choose URF files easily like loading in my plane

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Defining position using Euler Angles

startPos = [0, 0, 0]  # starting coordinates

startEuler = [0, 0, 0]  # [Roll,Pitch,Yaw] in radians

# Convert Euler Angles to quaternion for Pybullet

startOrientation = p.getQuaternionFromEuler(startEuler)

# Choosing what URF file to load in, I am loading the plane

planeId = p.loadURDF("plane.urdf", startPos, startOrientation)

# Add a shape

halfExtents = [0.5, 0.5, 0.5]
boxCollisionsShape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents)
boxVisualShape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents, rgbaColor=[1, 0, 0, 1])
box_StartPos = [2, 0, 0.5]  # starting position of the box
box_Start_Euler = [0, 0, 0]
box_Orientation = p.getQuaternionFromEuler(box_Start_Euler)  # Box start Orientation
boxID = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=boxCollisionsShape,
                          baseVisualShapeIndex=boxVisualShape,
                          basePosition=box_StartPos,
                          baseOrientation=box_Orientation)
# Add Camera Crap

cam1_coordinates = [0, 0, 0]
cam1_target = box_StartPos

# Camera Settings
fov = 60  # Field of view
aspect = 1  # Aspect ratio
near = 0.1  # Near clipping plane
far = 100  # Far clipping plane

# Matrices for 1 camera
view_matrix1 = p.computeViewMatrix(cam1_coordinates, cam1_target, [0, 0, 1])
projection_matrix1 = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# Need something to keep my plane running without shutting off
while True:
    p.stepSimulation()
    width, height, rgba_img1, depth_img1, seg_img1 = p.getCameraImage(width=640, height=480,
                                                                      viewMatrix=view_matrix1,
                                                                      projectionMatrix=projection_matrix1)[:5]
    # Convert depth image to numpy array
    depth_img1 = np.reshape(depth_img1, (height, width))

    # Extract the depth value at the center of the image
    center_depth_value = depth_img1[height // 2, width // 2]

    # Convert the depth value from normalized to actual distance
    distance = near + (far - near) * center_depth_value

    print(f"Distance to the box: {distance:.2f} meters")

    # time.sleep(2) # This gives me the time intervals between measurements, but it makes my simulation super slow
    # I need to figure out a way of not making it super buggy & shit
