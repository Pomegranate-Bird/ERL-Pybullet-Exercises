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
r2d2_coordinates = [0, 0, 0.5]  # Make sure it's on the ground
r2d2_orientation = [0, 0, 0]  # Neutral orientation (Euler Angles)
initial_r2d2_orientation = p.getQuaternionFromEuler(r2d2_orientation)

# Load the r2d2 model
r2d2Id = p.loadURDF("r2d2.urdf", r2d2_coordinates, initial_r2d2_orientation)


# Let's control the movement of the R2D2 through the UP,DOWN,RIGHT,LEFT keys

# define a function moving to point & i want to call all these already built functions to move me to that point

def move_forward():
    maxForce = 500
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=2,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=10,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=3,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=10,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=6,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=10,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=7,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=10,
                            force=maxForce)


def move_backwards():
    maxForce = 1000
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=2,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-10,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=3,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-10,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=6,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-10,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=7,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-10,
                            force=maxForce)


# turn_degrees = 360
# turn_radians = np.deg2rad(turn_degrees)

def turn_left():
    maxForce = 1000
    turn_velocity = 5.0  # Velocity for turning

    # Rotate left wheels backward
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=2,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-turn_velocity,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=3,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-turn_velocity,
                            force=maxForce)
    # Rotate right wheels forward
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=6,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=turn_velocity,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=7,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=turn_velocity,
                            force=maxForce)


def turn_right():
    maxForce = 1000
    turn_velocity = 5.0  # Velocity for turning

    # Rotate left wheels backward
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=2,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=turn_velocity,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=3,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=turn_velocity,
                            force=maxForce)
    # Rotate right wheels forward
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=6,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-turn_velocity,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=7,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=-turn_velocity,
                            force=maxForce)



    maxForce = 0
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=2,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=3,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=6,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=r2d2Id, jointIndex=7,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=maxForce)


def on_press(key):
    try:
        if key.char == 't':
            move_forward()
        elif key.char == 'y':
            move_backwards()
        elif key.char == 'i':
            turn_left()
        elif key.char == 'u':
            turn_right()
    except AttributeError:
        pass


def on_release(key):
    stop()


# Enable real-time simulation to interact with objects
p.setRealTimeSimulation(1)  # Must be set before starting the main loop

# Start listening for keyboard events
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    # Loop to keep the simulation running without closing unexpectedly
    while True:
        p.stepSimulation()
        time.sleep(1 / 240)
