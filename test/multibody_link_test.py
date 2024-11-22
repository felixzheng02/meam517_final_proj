import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane
p.loadURDF("plane.urdf")

# Base (fixed)
baseCollisionShape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
baseVisualShape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
baseMass = 0  # Fixed base

# Moving link (connected via prismatic joint)
linkCollisionShape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
linkVisualShape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
linkMass = 1
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]

robotId = p.createMultiBody(baseMass, baseCollisionShape, baseVisualShape, [0, 0, 0.1], [0, 0, 0, 1],
                            linkMasses=[linkMass],
                            linkCollisionShapeIndices=[linkCollisionShape],
                            linkVisualShapeIndices=[linkVisualShape],
                            linkPositions=[[0, 0, 0.3]],
                            linkOrientations=[[0, 0, 0, 1]],
                            linkParentIndices=[0],
                            linkInertialFramePositions=linkInertialFramePositions,
                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                            linkJointTypes=[p.JOINT_PRISMATIC],
                            linkJointAxis=[[0, 0, 1]])  # Motion along Z-axis



for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)
    p.setJointMotorControl2(bodyUniqueId=robotId,
                        jointIndex=0,
                        controlMode=p.TORQUE_CONTROL,  # Use TORQUE_CONTROL for both torques and forces
                        force=30)  # Apply a force of 10N along the joint axis
p.disconnect()