import pybullet as p
import pybullet_data
import time
import math

# Initialize physics client
physicsClient = p.connect(p.GUI)

# Set additional search path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.8)

# Load plane
planeId = p.loadURDF("plane.urdf")

# Create cube
cubeStartPos = [0, 0, 0.05]  # Positioned so that it rests on the ground
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

cubeCollisionShapeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05])
cubeVisualShapeId = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05], rgbaColor=[1, 0, 0, 1])

cubeMass = 1  # Mass of the cube
cubeId = p.createMultiBody(cubeMass, cubeCollisionShapeId, cubeVisualShapeId, cubeStartPos, cubeStartOrientation)

# Create robot hand
handStartPos = [-1.0, -1.0, 0.05]  # Positioned so that it rests on the ground
handStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

handCollisionShapeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.2, 0.05])
handVisualShapeId = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.2, 0.05], rgbaColor=[0, 1, 0, 1])

handMass = 1  # Mass of the hand
handId = p.createMultiBody(handMass, handCollisionShapeId, handVisualShapeId, handStartPos, handStartOrientation)

# Simulation loop
for i in range(10000):
    time.sleep(1./240.)
    p.stepSimulation()
    
    # Get positions and orientations
    handPos, handOrn = p.getBasePositionAndOrientation(handId)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(cubeId)
    
    # Reset the hand's Z position and velocity to keep it on the ground
    handPos = [handPos[0], handPos[1], 0.05]
    p.resetBasePositionAndOrientation(handId, handPos, handOrn)
    
    handLinearVel, handAngularVel = p.getBaseVelocity(handId)
    handLinearVel = [handLinearVel[0], handLinearVel[1], 0.0]
    p.resetBaseVelocity(handId, handLinearVel, handAngularVel)
    
    # Define target position and orientation
    targetPos = cubePos  # Target is the cube's current position
    targetAngle = i * 0.01  # Rotate over time
    
    # Compute desired velocity towards the target position
    kp_pos = 1.0  # Proportional gain for position
    kp_ang = 1.0  # Proportional gain for angle
    
    vel_x = kp_pos * (targetPos[0] - handPos[0])
    vel_y = kp_pos * (targetPos[1] - handPos[1])
    vel_z = 0.0  # No movement in Z-axis
    
    # Compute desired angular velocity towards the target angle
    handEuler = p.getEulerFromQuaternion(handOrn)
    angle_diff = targetAngle - handEuler[2]
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize angle difference
    
    ang_vel_z = kp_ang * angle_diff
    ang_vel = [0, 0, ang_vel_z]
    
    # Set the hand's linear and angular velocities
    p.resetBaseVelocity(handId, [vel_x, vel_y, vel_z], ang_vel)
