from estimator import Estimator
from controller import Controller
import numpy as np

class Robot:

    def __init__(self, x0, y0, theta0, w0, l, w_lim, K, theta_o_s, sh_s):
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.w = w0
        self.fw = np.array(w0[:2])
        self.fr = self.world_to_robot_frame(w0[:2])
        self.torque = w0[2]
        self.l = l
        self.estimator = Estimator(l, 0.1, 0.1)
        self.controller = Controller()
        self.theta_o_s = theta_o_s
        self.sh_s = sh_s
        self.w_lim = w_lim
        self.K = K
        self.xo = None
        self.yo = None
        self.d = None
        self.sh = 0

    def update_pose(self, x, y, theta):
        """theta: radian"""
        self.x = x
        self.y = y
        self.theta = theta
        self.fr = self.world_to_robot_frame(self.w[:2])

    def estimate(self):
        hand_mu, ground_mu = self.estimator.est_mu(self.fr[0], self.fr[1], self.fw[0], self.fw[1])
        self.xo, self.yo, self.d, self.sh = self.estimator.kinemitcs_est(np.array([self.x, self.y]), self.theta)
        return hand_mu, ground_mu, self.xo, self.yo, self.d, self.sh

    def joint_estimate_control(self, i):
        # estimation
        hand_mu, ground_mu = self.estimator.est_mu(self.fr[0], self.fr[1], self.fw[0], self.fw[1])
        hand_sliding, hand_pivoting, ground_sliding = self.estimator.contact_mode_est(self.fr[0], self.fr[1], self.torque, self.fw[0], self.fw[1])

        # if ground_sliding:
        #     # restart
        #     self.xo = None
        #     self.yo = None
        #     self.d = None
        #     self.sh = 0
        #     return self.w
        # elif not hand_pivoting:
        #     self.xo, self.yo, self.d, self.sh = self.estimator.kinemitcs_est()
        # else: # hand pivoting
            # TODO we know xo, yo, can estimate sh
        self.xo, self.yo, self.d, self.sh = self.estimator.kinemitcs_est(np.array([self.x, self.y]), self.theta)

        # control
        delta_f, delta_torque, delta_x_tar = self.controller.control(self.theta_o_s[i], self.sh_s[i], self.d, self.sh, self.fw, self.fr, self.torque, hand_mu, ground_mu, self.theta, self.l, self.w_lim, self.K)
        # print(f"delta_f={delta_f}, delta_torque={delta_torque}, delta_x_tar={delta_x_tar}")
        self.update_w(delta_f, delta_torque[0])
        return self.w
    
    def update_w(self, delta_f, delta_torque):
        self.w[:2] += delta_f
        self.w[2] += delta_torque
        self.fw = self.w[:2]
        self.fr = self.world_to_robot_frame(self.w[:2])
        self.torque = self.w[2]

    def get_pos(self):
        return [self.x, self.y]
    
    def world_to_robot_frame(self, input):
        """input: dim=2"""
        input = np.array(input).reshape([2, 1])
        return (np.array([[-np.cos(self.theta), np.sin(self.theta)],
                         [-np.sin(self.theta), -np.cos(self.theta)]]) @ input).reshape(2)