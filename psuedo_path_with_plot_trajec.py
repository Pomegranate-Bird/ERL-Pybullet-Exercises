import numpy as np
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt

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
init_car_orientation = [0, 0, np.degrees(-100)]
init_car_orientation_quat = p.getQuaternionFromEuler(init_car_orientation)
car_id = p.loadURDF("racecar/racecar.urdf", init_car_coordinates, init_car_orientation_quat)

# Define line (2D points, ignoring z)
points = [
    np.array([1, 1]),
    np.array([1, 2]),
    np.array([1, 3]),
    np.array([2, 3.5]),
    np.array([3, 3.5]),
    np.array([4, 4])
]

# Joint indices for the front and rear wheels
steering_joints = [4, 6]  # Front left and right wheel steering joints
rear_wheels = [2, 3]  # Rear left and right wheel driving joints
front_wheels = [5, 7]  # Set the initial steering angle and velocity

# Controller gains
Kp_steering = 1.0  # Proportional gain for steering
Kp_velocity = 2.0  # Proportional gain for velocity

# Tolerance for reaching the target
tolerance = 0.1

# Enable real-time simulation
p.setRealTimeSimulation(1)

# List to store trajectory points
trajectory = []


def get_car_position_and_orientation():
    pos, orn = p.getBasePositionAndOrientation(car_id)
    return np.array(pos[:2]), p.getEulerFromQuaternion(orn)  # Return only 2D position and orientation


def move_to_target(target_point):
    while True:
        car_position, car_orientation = get_car_position_and_orientation()
        car_heading = car_orientation[2]  # Yaw angle

        # Record the current position for trajectory
        trajectory.append(car_position)

        # Calculate the error in position
        position_error = target_point - car_position
        distance_error = np.linalg.norm(position_error)  # Distance from current position to target

        # Check if we have reached the target position
        if distance_error < tolerance:
            stop()
            break

        # Calculate the desired heading angle
        desired_heading = np.arctan2(position_error[1], position_error[0])

        # Error in heading
        heading_error = desired_heading - car_heading

        # Calculate the steering using the proportional controller
        steering_angle = Kp_steering * heading_error

        # Set the steering angle
        p.setJointMotorControlArray(
            bodyUniqueId=car_id,
            jointIndices=steering_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=[steering_angle, steering_angle])

        # Calculate the velocity using a proportional controller
        velocity = Kp_velocity * distance_error

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

        p.stepSimulation()
        time.sleep(1 / 240)


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


# Main loop to move through the points in the line
for point in points:
    move_to_target(point)

# Stop the car after final waypoint
stop()

# Convert trajectory to numpy array for plotting
trajectory = np.array(trajectory)

# Plot the trajectory
plt.figure()
plt.plot(trajectory[:, 0], trajectory[:, 1], label='Trajectory')
plt.plot([point[0] for point in points], [point[1] for point in points], 'ro', label='Waypoints')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Trajectory of the Car')
plt.legend()
plt.grid()
plt.show()
