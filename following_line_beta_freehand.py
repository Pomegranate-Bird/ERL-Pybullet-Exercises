import numpy as np
import pybullet as p
import pybullet_data
import time
from pynput import keyboard
import threading

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

# target_position = np.array([5, 5, 0], [], [], [], []) need to have a about 10 points to make my line
# for a line I want a series of points, one meter a way from each other or a box away. I need to loop it to check
#  3. if I'm some acceptable distance away from the point I (distance error-low enough) I move onto the next point
# I want to loop 3. until It reaches the last point then I want to break the loop once we're some distance error
# close to it
# Controller gains

p.setRealTimeSimulation(1)

Kp_steering = 0.5  # Proportional gain for steering
Kp_velocity = 1.0  # Proportional gain for velocity

# creating the line I want my robot to follow, Creating a list of lists containing my 1D vectors
p1 = [0, 0, 0]
p2 = [1, 1, 0]
p3 = [2, 2, 0]
p4 = [3, 3, 0]
p5 = [4, 4, 0]
p6 = [5, 5, 0]

target_line = [p1, p2, p3, p4, p5, p6]


def get_car_position_and_orientation():
    pos, orn = p.getBasePositionAndOrientation(car_id)  # passes the position & then the orientation
    # print(f"this is {p.getEulerFromQuaternion(orn)}") # printing the f(x) that gives me the quaternion in euler format
    return np.array(pos), p.getEulerFromQuaternion(orn)


def move_to_p2():
    while move_to_p2:  # looping the function to continuously update orn,pos
        car_position, car_orientation = get_car_position_and_orientation()
        car_heading = car_orientation[2]  # Yaw angle

        # Calculate the error in position
        position_error = p2 - car_position
        distance_error = np.linalg.norm(position_error[:2])  # norm = mag of vector, distance from origin to point

        # Calculating desired heading angle
        desired_heading = np.arctan2(car_position[1], car_position[2])

        # Error in heading
        heading_error = desired_heading * car_heading

        # Calculate the steering using the proportional controller
        steering_angle = Kp_steering * heading_error

        # Set the steering angle
        p.setJointMotorControlArray(
            bodyUniqueId=car_id,
            jointIndices=steering_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=[steering_angle, steering_angle])

        # Calculate the velocity using a proportional controller
        velocity = (Kp_velocity * distance_error)

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


while True:
    p.stepSimulation()
    move_to_p2()
    time.sleep(1 / 240)
