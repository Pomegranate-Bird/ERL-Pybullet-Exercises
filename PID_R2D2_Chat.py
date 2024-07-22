import numpy as np
import pybullet as p
import pybullet_data
import time
from pynput import keyboard

# Connect to PyBullet
p.connect(p.GUI)

# Load the URDF file for the car
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the ground plane
coordinates = [0, 0, 0]
orientation = [0, 0, 0]  # Euler Angles Roll, Pitch, Yaw
sim_orientation = p.getQuaternionFromEuler(orientation)
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf", coordinates, sim_orientation)

# Load the car model
init_car_coordinates = [0, 0, 0.2]
init_car_orientation = [0, 0, 0]
init_car_orientation_quat = p.getQuaternionFromEuler(init_car_orientation)
car_id = p.loadURDF("racecar/racecar.urdf", init_car_coordinates, init_car_orientation_quat)

# Joint indices for the front and rear wheels
steering_joints = [4, 6]  # Front left and right wheel steering joints
rear_wheels = [2, 3]  # Rear left and right wheel driving joints
front_wheels = [5, 7]  # Set the initial steering angle and velocity

# Steering angle in radians (positive for left turn, negative for right turn)
steering_angle = 0
velocity = 10  # Desired velocity for the rear wheels

# Enable real-time simulation
p.setRealTimeSimulation(1)


def move_backward():
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=rear_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[-velocity, -velocity])
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=front_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[-velocity, -velocity])


def turn_left():
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=steering_joints,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[180, 180])


def turn_right():
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=steering_joints,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[-180, -180])


def move_forward():
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=steering_joints,
        controlMode=p.POSITION_CONTROL,
        targetPositions=([0, 0]))

    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=front_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[velocity, velocity])

    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=rear_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[velocity, velocity])


def on_press(key):
    try:
        if key.char == 't':
            move_forward()
        elif key.char == 'y':
            move_backward()
        elif key.char == 'i':
            turn_left()
        elif key.char == 'u':
            turn_right()
    except AttributeError:
        pass


"""while True:
    # Apply steering to the front wheels
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=steering_joints,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[steering_angle, steering_angle]
    )

    # Apply velocity to the rear wheels
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=rear_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[velocity, velocity])"""


def stop():
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=rear_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[0, 0])
    p.setJointMotorControlArray(
        bodyUniqueId=car_id,
        jointIndices=front_wheels,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[0, 0])


def on_release(key):
    stop()


with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    # Loop to keep the simulation running without closing unexpectedly
    while True:
        p.stepSimulation()
        time.sleep(1 / 240)
