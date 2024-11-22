from robot import Robot
import numpy as np
import pybullet as p
import pybullet_data
import time
import math


class Simulation:

    def __init__(self):
        pass

    def run(self, delta_theta_o_s, delta_sh_s):
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
        
        robotStartPos = [0, -0.04, .23]
        # robotStartOrientation = [0, 0, 0]
        robotStartOrientation = cubeStartOrientation
        w0 = [0, -90, 12]
        robotId = p.loadURDF("src/urdf/robot.urdf", 
                             basePosition=robotStartPos, 
                             baseOrientation=robotStartOrientation, 
                             useFixedBase=True)

        w_lim = np.array([200, 200, 200])
        K = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, .5]])
        robot = Robot(robotStartPos[1], robotStartPos[2], orientation[0], w0, 0.1, w_lim, K, delta_theta_o_s, delta_sh_s)

        cubeFriction = p.getDynamicsInfo(cubeId, -1)[1]
        planeFriction = p.getDynamicsInfo(planeId, -1)[1]
        robotFriction = p.getDynamicsInfo(robotId, -1)[1]
        print("Cube friction coefficient:", cubeFriction)
        print("Plane friction coefficient:", planeFriction)
        print("Cube friction coefficient:", robotFriction)

        steps = len(delta_theta_o_s)
        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, force=0)

        # p.enableJointForceTorqueSensor(robotId, 0)
        w = w0
        for i in range(50): # wait till hand in line contact with object
            hand_pos = p.getLinkState(robotId, 2)[0]
            hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
            robot.update_pose(hand_pos[1], hand_pos[2], hand_orientation[0])
            p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
            p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
            p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])
            p.stepSimulation()
            time.sleep(1./240.)
        # robot.sh = 0
        for i in range(steps): # control
            print(i)
            hand_pos = p.getLinkState(robotId, 2)[0]
            hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
            print(hand_orientation[0])
            robot.update_pose(hand_pos[1], hand_pos[2], hand_orientation[0])
            w = robot.joint_estimate_control(i)
            print(w)
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
            
            

        p.disconnect()


simulation = Simulation()
sh_s = [0] * 100
theta_o_s = [.5] * 100
simulation.run(theta_o_s, sh_s)






        # for i in range(100): # warm up
        #     hand_pos = p.getLinkState(robotId, 2)[0]
        #     hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
        #     robot.update(hand_pos[1], hand_pos[2], hand_orientation[0])
        #     hand_mu, ground_mu, xo, yo, d, sh = robot.estimate()
        #     # with open("output.txt", "a") as f:
        #     #     print(f"hand_mu={hand_mu}, ground_mu={ground_mu}, xo={xo}, yo={yo}, d={d}, sh={sh}", file=f)
        #     p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
        #     p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
        #     p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])
        #     p.stepSimulation()
        #     time.sleep(1./240.)