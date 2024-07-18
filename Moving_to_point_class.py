import numpy as np
import pybullet as p
import pybullet_data
import time
from pynput import keyboard
import threading
import scipy.integrate as integrate
import scipy.special as special


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


# Controller gains
Kp_steering = 0.5  # Proportional gain for steering
Kp_velocity = 1.0  # Proportional gain for velocity


# Load the car as a flag
flag_position = [5, 5, 0]  # Set the flag position
flag_orientation = [0, 0, 0]
flag_orientation_quat = p.getQuaternionFromEuler(flag_orientation)
flag_id = p.loadURDF("racecar/racecar.urdf", flag_position, flag_orientation_quat)


# Desired target position for the car to drive to
target_position = np.array([5, 5, 0])  # 1D array representing a vector, so the position I want to get to in the sim


# Enable real-time simulation
p.setRealTimeSimulation(1)


def get_car_position_and_orientation():
    pos, orn = p.getBasePositionAndOrientation(car_id)  # passes the position & then the orientation
    # print(f"this is {p.getEulerFromQuaternion(orn)}") # printing the f(x) that gives me the quaternion in euler format
    return np.array(pos), p.getEulerFromQuaternion(orn)
    # ^^^^
    # returns a vector [x,y,z] & a list of floating points


def move_to_target():
    while moving_to_target_flag:  # A loop to continue f(x) updating pos,orn
        car_position, car_orientation = get_car_position_and_orientation()
        car_heading = car_orientation[2]  # Yaw angle

        # Calculate the error in position
        position_error = target_position - car_position
        print(f"This is {position_error[:2]}")
        distance_error = np.linalg.norm(position_error[:2])        # norm = mag of vector, distance from origin to point

        # Calculate the desired heading angle
        desired_heading = np.arctan2(position_error[1], position_error[0])

        # Calculate the error in heading
        heading_error = desired_heading - car_heading

        # Calculate the steering angle using a proportional controller
        steering_angle = Kp_steering * heading_error

        # Set the steering angle
        p.setJointMotorControlArray(
            bodyUniqueId=car_id,
            jointIndices=steering_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=[steering_angle, steering_angle])

        # Calculate the velocity using a proportional controller
        velocity = (Kp_velocity * distance_error)
        # I need to add the integral of the error in respect to time

        # Set the wheel velocities
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

        # Sleep for a short duration to control the loop frequency
        time.sleep(1 / 240)


def stop():
    global moving_to_target_flag
    moving_to_target_flag = False
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


def on_press(key):
    global moving_to_target_flag
    try:
        if key.char == 'm':
            if not moving_to_target_flag:
                moving_to_target_flag = True
                threading.Thread(target=move_to_target).start()
    except AttributeError:
        pass


def on_release(key):
    stop()


# Initialize the moving to target flag
moving_to_target_flag = False

# Start the keyboard listener
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    # Loop to keep the simulation running
    while True:
        p.stepSimulation()
        time.sleep(1 / 240)
