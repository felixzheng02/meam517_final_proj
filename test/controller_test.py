import sys
sys.path.insert(0, '/Users/felix/Desktop/meam5170/final_project/src')
from controller import Controller
from utils import world_to_robot_frame, plot_w_hist, plot_cost_hist
import numpy as np
import pybullet as p
import pybullet_data
import time
import math
# import keyboard


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

orientation = [np.pi/5, 0, 0]
# Create cube
cubeStartPos = [0, 0, .1]
cubeSize = [.1, .1, .1]
cubeStartOrientation = p.getQuaternionFromEuler(orientation)
cubeCollisionShapeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=cubeSize)
cubeVisualShapeId = p.createVisualShape(p.GEOM_BOX, halfExtents=cubeSize, rgbaColor=[1, 0, 0, 1])
cubeMass = 10
cubeId = p.createMultiBody(cubeMass, cubeCollisionShapeId, cubeVisualShapeId, cubeStartPos, cubeStartOrientation)

robotStartPos = [0, -0.04, .2]
# robotStartOrientation = [0, 0, 0]
robotStartOrientation = cubeStartOrientation
w0 = [0, -90, 1]
robotId = p.loadURDF("src/urdf/robot.urdf", 
                        basePosition=robotStartPos, 
                        baseOrientation=robotStartOrientation, 
                        useFixedBase=True)

w_lim = np.array([200, 200, 200])
K = 1000000*np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])

theta_s = [.5]*100
sh_s = [0]*100
controller = Controller()
p.changeDynamics(cubeId, -1, lateralFriction=1)
p.changeDynamics(robotId, -1, lateralFriction=1)

cubeFriction = p.getDynamicsInfo(cubeId, -1)[1]
planeFriction = p.getDynamicsInfo(planeId, -1)[1]
robotFriction = p.getDynamicsInfo(robotId, -1)[1]
print("Cube friction coefficient:", cubeFriction)
print("Plane friction coefficient:", planeFriction)
print("Cube friction coefficient:", robotFriction)

steps = len(theta_s)
p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, force=0)

# p.enableJointForceTorqueSensor(robotId, 0)
w = w0
w_hist = []
cost_hist = []
# for i in range(50): # wait till hand in line contact with object
#     p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
#     p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
#     p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])
#     p.stepSimulation()
#     time.sleep(1./240.)
# robot.sh = 0
try:
    for i in range(steps): # control
        # print(i)
        hand_pos = p.getLinkState(robotId, 2)[0]
        hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
        theta = hand_orientation[0]
        # print(theta)
        delta_f, delta_torque, delta_x_tar, optimal_cost = controller.control(theta_s[i], sh_s[i], .1, 0, w[:2], world_to_robot_frame(w[:2], theta), w[2], 1, 1, theta, .05, np.array([200, 200, 200]), K)
        print(f"delta_f={delta_f}, delta_torque={delta_torque}, delta_x_tar={delta_x_tar}")
        w[:2] += delta_f
        w[2] += delta_torque[0]
        # print(w)
        w_hist.append(w.copy())
        cost_hist.append(optimal_cost)
        p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
        p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])
        p.stepSimulation()
        time.sleep(1./240.)

        

        # joint_state_0 = p.getJointState(robotId, 0)
        # joint_state_1 = p.getJointState(robotId, 1)
        # joint_state_2 = p.getJointState(robotId, 2)
        # print(f"joint positions = {joint_state_0[0]}, {joint_state_1[0]}, {joint_state_2[0]}")
        # print(f"joint forces = {joint_state_0[2]}, {joint_state_1[2]}, {joint_state_2[2]}")
except KeyboardInterrupt:
    p.disconnect()

# print(w_hist)
plot_w_hist(np.array(w_hist))
plot_cost_hist(np.array(cost_hist))