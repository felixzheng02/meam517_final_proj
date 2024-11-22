from livestats import livestats
import numpy as np
from scipy.optimize import lsq_linear

class Estimator:
    """
    get_wrench_constraints(self): return wrench constraints
    contact_mode_est(w): given wrench, check constraints to estimate contact mode
    kinematic_est(): return ground contact position
    est_mu(w): estimate friction coefficients based on new wrench and previous estimation
    get_hand_mu()
    get_ground_mu()
    """

    def __init__(self, l, mu_est_eps, mode_est_eps):
        self.l = l
        self.hand_mu_est = livestats.LiveStats([0.99])
        self.ground_mu_est = livestats.LiveStats([0.99])
        self.mu_est_eps = mu_est_eps
        self.mode_est_eps = mode_est_eps
        # self.w_hist = []
        self.rh_hist = [] # [n, 2]
        self.theta_hist = [] # [n,]

    def get_wrench_constraints(self):
        return
    
    def contact_mode_est(self, ft, fn, torque, fi, fj):
        """
        input: wrench
        output: (bool, bool, bool)
        """
        hand_sliding = False
        hand_pivoting = False
        hand_mu = self.get_hand_mu()
        # check hand sliding
        if hand_mu*fn + ft < self.mode_est_eps or hand_mu*fn - ft < self.mode_est_eps:
            hand_sliding = True
        # check hand pivoting
        if self.l*fn - torque < self.mode_est_eps or self.l*fn + torque < self.mode_est_eps:
            hand_pivoting = True
        
        ground_sliding = False
        f_grav_j, f_grav_i = -9.81, 0
        ground_mu = self.get_ground_mu()
        # check ground sliding
        if -ground_mu * (fj+f_grav_j) + (fi+f_grav_i) < self.mode_est_eps or -ground_mu * (fj+f_grav_j) - (fi+f_grav_i) < self.mode_est_eps:
            ground_sliding = True
        
        return hand_sliding, hand_pivoting, ground_sliding
    
    def kinemitcs_est(self, rh, theta):
        """rh: np.array([xh, yh])"""
        self.add_rh(list(rh))
        self.add_theta(theta)
        # hand line contact
        d, xo, yo = self.regress(np.array(self.theta_hist), np.array(self.rh_hist))
        e_ht = np.array([-np.cos(theta), -np.sin(theta)])
        sh = np.inner((np.array([xo, yo]) - rh), e_ht)
        # other contact modes
        return xo, yo, d, sh
    
    def regress(self, thetas, rh):
        """
        input: thetas: [n,]
        r_h: [n, 2] = (x_h, y_h), n: number of samples
        regress d, x_o, y_o
        """
        n = thetas.shape[0]
        thetas = thetas.reshape([n, 1])
        e_hn = np.hstack([np.sin(thetas), -np.cos(thetas)]) # [n, 2]

        A = np.hstack([-np.ones([n, 1]), np.sin(thetas), -np.cos(thetas)])
        b = np.sum(rh * e_hn, axis=1)
        result = lsq_linear(A, b, bounds=[[-np.inf, np.inf], [-1, 1], [-1, 1]])
        # result, _, _, _ = np.linalg.lstsq(A, b, None)
        print(result['x'])
        d, xo, yo = result.flatten()
        return d, xo, yo
    
    def est_mu(self, ft, fn, fi, fj):
        """w: wrench [6,]"""
        # estimate hand contact friction constant
        self.hand_mu_est.add(np.abs(ft)/(np.abs(fn)+self.mu_est_eps))
        est_hand_mu = self.hand_mu_est.mean()

        # estimate ground contact friction constant
        self.ground_mu_est.add(np.abs(fi)/(np.abs(fj)+self.mu_est_eps))
        est_ground_mu = self.ground_mu_est.mean()

        return est_hand_mu, est_ground_mu
    
    def add_rh(self, r_h: list):
        """r_h: [2]"""
        self.rh_hist.append(r_h)

    def add_theta(self, theta):
        self.theta_hist.append(theta)
    
    def get_hand_mu(self):
        return self.hand_mu_est.mean()
    
    def get_ground_mu(self):
        return self.ground_mu_est.mean()