import sys
sys.path.insert(0, '/Users/felix/Desktop/meam5170/final_project/src')
from robot import Robot
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
p.changeDynamics(objId, -1, lateralFriction=.1)
p.changeDynamics(robotId, -1, lateralFriction=10)
p.changeDynamics(planeId, -1, lateralFriction=10)
objFriction = p.getDynamicsInfo(objId, -1)[1]
planeFriction = p.getDynamicsInfo(planeId, -1)[1]
robotFriction = p.getDynamicsInfo(robotId, -1)[1]
print("obj friction coefficient:", objFriction)
print("Plane friction coefficient:", planeFriction)
print("obj friction coefficient:", robotFriction)

# robot parameters
n = 1000
theta_s = [0]*n
sh_s = [0]*n
w_lim = np.array([200, 200, 200])
K = 10*np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
w = [0, -10, 0]
robot = Robot(robotStartPos[1], 
              robotStartPos[2], 
              0,
              w, 
              0.1, 
              w_lim, 
              K, 
              theta_s, 
              sh_s)
w_s = []

try:
    for i in range(n):
        print(i)
        hand_pos = p.getLinkState(robotId, 2)[0]
        pos = np.array(hand_pos[:2])
        hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
        theta = hand_orientation[0]

        robot.update_pose(pos[0], pos[1], theta)
        w = robot.joint_estimate_control(i)
        w_s.append(w)

        # print(f"y={w[0]:.3f}, z={w[1]:.3f}, torque={w[2]:.3f}")
        
        p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
        p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    p.disconnect()

plot(np.array(w_s)[:, :2], 2, ['force_y', 'force_z'])
plot(np.array(w_s)[:, 2], 1, ['torque'])