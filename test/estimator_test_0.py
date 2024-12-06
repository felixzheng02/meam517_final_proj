import sys
sys.path.insert(0, '/Users/felix/Desktop/meam5170/final_project/src')
from estimator import Estimator
from utils import world_to_robot_frame, plot
import numpy as np
import pybullet as p
import pybullet_data
import time


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# simulation entities
planeId = p.loadURDF("plane.urdf")

objStartPos = [0, 0, .1]
objStartOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])
objCollisionShapeId = p.createCollisionShape(p.GEOM_MESH, fileName='src/object0.obj', meshScale=[.1, .1, .1])
objVisualShapeId = p.createVisualShape(p.GEOM_MESH, fileName='src/object0.obj', meshScale=[.1, .1, .1], rgbaColor=[1, 0, 0, 1])
objMass = 1
objId = p.createMultiBody(objMass, objCollisionShapeId, objVisualShapeId, objStartPos, objStartOrientation)

robotStartPos = [0, 0, .2]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("src/urdf/robot.urdf", 
                        basePosition=robotStartPos, 
                        baseOrientation=robotStartOrientation, 
                        useFixedBase=True)
p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, force=0)

# simulation parameters
p.changeDynamics(objId, -1, lateralFriction=10)
p.changeDynamics(robotId, -1, lateralFriction=10)
p.changeDynamics(planeId, -1, lateralFriction=1)
objFriction = p.getDynamicsInfo(objId, -1)[1]
planeFriction = p.getDynamicsInfo(planeId, -1)[1]
robotFriction = p.getDynamicsInfo(robotId, -1)[1]
print("obj friction coefficient:", objFriction)
print("Plane friction coefficient:", planeFriction)
print("obj friction coefficient:", robotFriction)

# robot parameters
w_lim = np.array([200, 200, 200])
K = 10*np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
ws = []
for i in range(50):
    ws.append([0, -100, 2])
for i in range(50):
    ws.append([100, -100, 15])
# for i in range(1000):
#     ws.append([50, -100, -5])

# for i in range(10):
#     for j in range(10):
#         ws.append([10, -10, 0])
#     for j in range(10):
#         ws.append([-15, -10, 0])

estimator = Estimator(.05, .1, .1)

steps = len(ws)-1
mu_s = []
xo_yo_d_sh_s = []

try:
    for i in range(steps):
        hand_pos = p.getLinkState(robotId, 2)[0]
        pos = np.array(hand_pos[:2])
        hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
        theta = hand_orientation[0]

        fr = world_to_robot_frame(ws[i][:2], theta)
        hand_mu, ground_mu = estimator.est_mu(fr[0], fr[1], ws[i][0], ws[i][1])
        mu_s.append([hand_mu, ground_mu])
        xo, yo, d, sh = estimator.kinemitcs_est(pos, theta)
        xo_yo_d_sh_s.append([xo, yo, d, sh])

        print(f"hand_mu={hand_mu:.3f},  ground_mu={ground_mu:.3f}")
        print(f"xo={xo:.3f}, yo={yo:.3f}, d={d:.3f}, sh={d:.3f}")
        
        p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=ws[i+1][0])
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=ws[i+1][1])
        p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=ws[i+1][2])
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    p.disconnect()

plot(np.array(mu_s), 2, ['hand_mu', 'ground_mu'])
plot(np.array(xo_yo_d_sh_s), 4, ['xo', 'yo', 'd', 'sh'])