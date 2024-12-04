import sys
sys.path.insert(0, '/Users/felix/Desktop/meam5170/final_project/src')
from controller import Controller
from utils import world_to_robot_frame, plot
import numpy as np
import pybullet as p
import pybullet_data
import time
import math
# import keyboard


def simulate(K):
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    planeId = p.loadURDF("plane.urdf")

    orientation = [np.pi/2, 0, 0]
    # Create obj
    objStartPos = [0, 0, .1]
    objSize = [.1, .1, .1]
    objStartOrientation = p.getQuaternionFromEuler(orientation)
    objCollisionShapeId = p.createCollisionShape(p.GEOM_MESH, fileName='src/object0.obj', meshScale=[.1, .1, .1])
    objVisualShapeId = p.createVisualShape(p.GEOM_MESH, fileName='src/object0.obj', meshScale=[.1, .1, .1], rgbaColor=[1, 0, 0, 1])
    objMass = 1
    objId = p.createMultiBody(objMass, objCollisionShapeId, objVisualShapeId, objStartPos, objStartOrientation)


    robotStartPos = [0, 0, .2]
    # robotStartOrientation = [0, 0, 0]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    w0 = [0, -10, 0]
    robotId = p.loadURDF("src/urdf/robot.urdf", 
                            basePosition=robotStartPos, 
                            baseOrientation=robotStartOrientation, 
                            useFixedBase=True)

    w_lim = np.array([200, 200, 200])

    n = 1000
    theta_s = [0]*n
    sh_s = [0]*n
    controller = Controller()
    p.changeDynamics(objId, -1, lateralFriction=.1)
    p.changeDynamics(robotId, -1, lateralFriction=10)
    p.changeDynamics(planeId, -1, lateralFriction=10)

    objFriction = p.getDynamicsInfo(objId, -1)[1]
    planeFriction = p.getDynamicsInfo(planeId, -1)[1]
    robotFriction = p.getDynamicsInfo(robotId, -1)[1]
    print("obj friction coefficient:", objFriction)
    print("Plane friction coefficient:", planeFriction)
    print("obj friction coefficient:", robotFriction)

    steps = len(theta_s)
    p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, force=0)

    # p.enableJointForceTorqueSensor(robotId, 0)
    w = w0
    w_hist = []
    delta_x_tar_hist = []
    cost_hist = []
    # for i in range(50): # wait till hand in line contact with object
    #     p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
    #     p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
    #     p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])
    #     p.stepSimulation()
    #     time.sleep(1./240.)
    # robot.sh = 0

    aabb_min, aabb_max = p.getAABB(objId)
    d = aabb_max[2] - aabb_min[2]
    # Visualization scaling factors
    force_scale = 10  # Scale for the force vector
    torque_scale = 0.05  # Scale for the torque vector
    try:
        for i in range(steps):
            p.removeAllUserDebugItems()

            obj_pos_ori = p.getBasePositionAndOrientation(objId)
            obj_pos = [obj_pos_ori[0][1], 0]
            obj_orientation = p.getEulerFromQuaternion(obj_pos_ori[1])[0] - orientation[0]
            # Get the axis-aligned bounding box (AABB) of the object
            
            sh_origin = [np.sin(obj_orientation)*d, np.cos(obj_orientation)*d]

            hand_pos = p.getLinkState(robotId, 2)[0][1:]
            hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
            theta = hand_orientation[0]
            sh = np.linalg.norm(np.array(hand_pos)-np.array(sh_origin))
            # print(sh)
            # print(theta)
            # try:
            delta_f, delta_torque, delta_x_tar, optimal_cost = controller.control(theta_s[i], 
                                                                                    sh_s[i], 
                                                                                    d=d, 
                                                                                    sh=sh, 
                                                                                    fw=w[:2], 
                                                                                    fr=world_to_robot_frame(w[:2], theta), 
                                                                                    torque=w[2], 
                                                                                    mu_h=.1, 
                                                                                    mu_g=10, 
                                                                                    theta=theta, 
                                                                                    l=.05, 
                                                                                    wrench_lim=np.array([200, 200, 200]), 
                                                                                    K=K, 
                                                                                    hand_sliding_constraint_on=True, 
                                                                                    hand_pivoting_constraint_on=True, 
                                                                                    ground_sliding_constraint_on=True, 
                                                                                    w_lim_on=True)
            # except:
            #     p.disconnect()
            #     return i
            delta_x_tar_hist.append(delta_x_tar)
            # print(f"time {i}: delta_f={delta_f}, delta_torque={delta_torque}, delta_x_tar={delta_x_tar}")
            w[:2] += delta_f
            w[2] += delta_torque[0]
            # print(w)
            w_hist.append(w.copy())
            cost_hist.append(optimal_cost)
            p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=w[0])
            p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=w[1])
            p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=w[2])

            # Draw force vector w[:2]
            force_start = p.getBasePositionAndOrientation(robotId)[0]
            fy_end = [
                force_start[0],
                force_start[1] + w[0] * force_scale,
                force_start[2]
            ]
            p.addUserDebugLine(force_start, fy_end, lineColorRGB=[0, 1, 0], lineWidth=10)
            fz_end = [
                force_start[0],
                force_start[1],
                force_start[2] + w[1] * force_scale
            ]
            p.addUserDebugLine(force_start, fz_end, lineColorRGB=[0, 0, 1], lineWidth=10)

            # # Draw torque vector w[2]
            # torque_start = force_start
            # torque_end = [
            #     torque_start[0],
            #     torque_start[1],
            #     torque_start[2] + w[2] * torque_scale
            # ]
            # p.addUserDebugLine(torque_start, torque_end, lineColorRGB=[0, 0, 1], lineWidth=3)

            if np.abs(theta-theta_s[i]) > np.pi/6:
                return i

            p.stepSimulation()
            time.sleep(1./240.)

            

            # joint_state_0 = p.getJointState(robotId, 0)
            # joint_state_1 = p.getJointState(robotId, 1)
            # joint_state_2 = p.getJointState(robotId, 2)
            # print(f"joint positions = {joint_state_0[0]}, {joint_state_1[0]}, {joint_state_2[0]}")
            # print(f"joint forces = {joint_state_0[2]}, {joint_state_1[2]}, {joint_state_2[2]}")
    except:
        p.disconnect()
        return i
    return i
    # print(w_hist)
    plot(np.array(w_hist), 3, ['fy', 'fz', 'torque'])
    plot(np.array(delta_x_tar_hist), 3, ['delta_x_tar_y', 'delta_x_tar_z', 'delta_x_tar_theta'])
    plot(np.array(cost_hist))


scalars = range(10, 6000, 100)
iter_s = []

for scalar in scalars:
    print(scalar)
    K = scalar*np.array([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
    iter = simulate(K)
    iter_s.append(iter)

plot(np.array(iter_s))