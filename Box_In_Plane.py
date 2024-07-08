# We will now add a Box into our Pybullet Simulation

# Importing the needed Python libraries for plane/box simulation
import pybullet as p
import pybullet_data

Connecting_to_pybullet = p.connect(p.GUI)

Gravity = p.setGravity(0, 0, -9.8)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

coordinates = [0, 0, 0]
oreintation = [0, 0, 0]

intial_oreintation = p.getQuaternionFromEuler(oreintation)

Plane = p.loadURDF("plane.urdf", coordinates, intial_oreintation)

# If the stuff above did not make sense, practice Plane_Exercise first,
# until you can set it up on your own

# Now we will build our Box
halfExtents = [0.5, 0.5, 0.5]  # Dimensions of our box

# Coordinates & Orientation of our box

box_coordinates = [0, 0, 0]
box_oreintation = [0, 0, 0]

# to make life simple use Quaternion's for pybullet
intial_box_orientation = p.getQuaternionFromEuler(box_oreintation)

# Create collision body
Collisionbody = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents)

# Create visionbody
visualBody = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents, rgbaColor=[1, 0, 0, 1])

# Create multibody
Multibody = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=Collisionbody,
                              baseVisualShapeIndex=visualBody,
                              basePosition=box_coordinates,
                              baseOrientation=intial_box_orientation)
while True:
    p.stepSimulation()
    p.setTimeStep(1 / 240)
