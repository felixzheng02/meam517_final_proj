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


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

objStartOrientation = [np.pi/2, 0, 0]
objStartPos = [0, 0, .1]
objSize = [.1, .1, .1]
objCollisionShapeId = p.createCollisionShape(p.GEOM_MESH, fileName='src/object0.obj', meshScale=[.1, .1, .1])
objVisualShapeId = p.createVisualShape(p.GEOM_MESH, fileName='src/object0.obj', meshScale=[.1, .1, .1], rgbaColor=[1, 0, 0, 1])
objMass = 1
objId = p.createMultiBody(objMass, objCollisionShapeId, objVisualShapeId, objStartPos, p.getQuaternionFromEuler(objStartOrientation))

robotStartPos = [0, 0, .2]
robotStartOrientation = [0, 0, 0]
p.getQuaternionFromEuler([0, 0, 0])
w0 = [0, -10, 0]
robotId = p.loadURDF("src/urdf/robot.urdf", 
                        basePosition=robotStartPos, 
                        baseOrientation=p.getQuaternionFromEuler(robotStartOrientation), 
                        useFixedBase=True)

w_lim = 300 * np.array([1, 1, 1])

K_scalar = 1000

K = K_scalar * np.array([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
steps = 10000
theta_s = [np.pi/30, -np.pi/30, np.pi/30, -np.pi/30, np.pi/30]
theta_tar_idx = 0
sh_s = [0]*steps
robot_aabb_min, robot_aabb_max = p.getAABB(robotId)
l = robot_aabb_max[1] - robot_aabb_min[1]
controller = Controller(l, 
                        w_lim, 
                        K, 
                        lambda_theta=1000, 
                        lambda_x_tar=1, 
                        hand_sliding_constraint_on=True, 
                        hand_pivoting_constraint_on=True, 
                        ground_sliding_constraint_on=True, 
                        w_lim_on=True,
                        momentum_control_on=False)
mu_h = 10
mu_g = 10
p.changeDynamics(objId, -1, lateralFriction=mu_h)
p.changeDynamics(robotId, -1, lateralFriction=mu_g)
p.changeDynamics(planeId, -1, lateralFriction=mu_g)

p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, force=0)

w = w0
w_hist = []
delta_x_tar_hist = []
cost_hist = []
theta_hist = []
theta_tar_hist = []

obj_aabb_min, obj_aabb_max = p.getAABB(objId)
d = obj_aabb_max[2] - obj_aabb_min[2]

force_scale = 10  # Scale for the force vector
torque_scale = 0.05  # Scale for the torque vector



try:
    for i in range(steps):
        p.removeAllUserDebugItems()

        obj_pos_ori = p.getBasePositionAndOrientation(objId)
        obj_pos = [obj_pos_ori[0][1], 0]
        obj_orientation = p.getEulerFromQuaternion(obj_pos_ori[1])[0] - objStartOrientation[0]
        # Get the axis-aligned bounding box (AABB) of the object
        
        sh_origin = [np.sin(obj_orientation)*d, np.cos(obj_orientation)*d]

        hand_pos = p.getLinkState(robotId, 2)[0][1:]
        hand_orientation = p.getEulerFromQuaternion(p.getLinkState(robotId, 2)[1])
        theta = hand_orientation[0]
        sh = np.linalg.norm(np.array(hand_pos)-np.array(sh_origin))
        # try:
        delta_f, delta_torque, delta_x_tar, optimal_cost = controller.control(theta_s[theta_tar_idx], 
                                                                                sh, 
                                                                                d=d, 
                                                                                sh=sh, 
                                                                                fw=w[:2], 
                                                                                fr=world_to_robot_frame(w[:2], theta), 
                                                                                torque=w[2], 
                                                                                mu_h=mu_h, 
                                                                                mu_g=mu_g, 
                                                                                theta=theta)
        # except:
        #     p.disconnect()

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

        theta_hist.append(theta)
        theta_tar_hist.append(theta_s[theta_tar_idx])

        if abs(theta - theta_s[theta_tar_idx]) < 1e-2:
            theta_tar_idx += 1
            if theta_tar_idx == len(theta_s):
                break
        #     input("Press ENTER to continue")

        p.stepSimulation()
        time.sleep(1./240.)
except:
    p.disconnect()

print(theta_tar_hist)
print("###########################")
print(theta_hist)
# plot(np.array([theta_hist, theta_tar_hist]).T, 2, ['theta', 'theta_tar'])
# plot(np.array(w_hist), 3, ['fy', 'fz', 'torque'])