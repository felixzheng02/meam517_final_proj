from pydrake.all import (MathematicalProgram, OsqpSolver)
from pydrake.symbolic import cos, sin, Expression
import numpy as np


class Controller:

    def __init__(self):
        pass

    def control(self, theta_tar, sh_tar, d, sh, fw, fr, torque, mu_h, mu_g, theta, l, wrench_lim, K):
        ft, fn = fr
        fi, fj = fw
        fw = np.array([fi, fj])
        wrench_lim = np.array(wrench_lim)
        if type(torque) == np.ndarray:
            torque = torque[0]

        v_theta = np.array([-d, sh, 1]).T # hand frame
        v_h = np.array([-1, 0, 0]) # hand frame
        v_g = np.array([1, 0, 0]) # world_frame

        delta_theta = theta_tar - theta
        delta_sh = sh_tar - sh

        prog = MathematicalProgram()
        delta_f = prog.NewContinuousVariables(2, 'delta_f') # [2,] world frame
        delta_f_r = self.world_to_robot_frame(delta_f, theta) # hand frame
        delta_torque = prog.NewContinuousVariables(1, 'delta_torque')
        delta_x_tar = prog.NewContinuousVariables(3, 'delta_x_tar') # hand frame

        prog.AddQuadraticCost((delta_x_tar.T @ v_theta - delta_theta) ** 2)
        # prog.AddQuadraticCost((delta_x_tar.T @ v_h - delta_sh) ** 2)
        
        f_grav_j = -9.81
        gammas = np.array([1, 1, 1, 1, 1, 1])
        # if delta_sh > 0:
        
        # hand sliding
        hand_sliding_constraint_0 = mu_h * (fn+gammas[0]*delta_f_r[1]) + (ft+gammas[0]*delta_f_r[0])
        hand_sliding_constraint_1 = mu_h * (fn+gammas[1]*delta_f_r[1]) - (ft+gammas[1]*delta_f_r[0])
        if delta_sh > 0:
            prog.AddConstraint(hand_sliding_constraint_0 == 0)
            prog.AddConstraint(hand_sliding_constraint_1 >= 0)
        elif delta_sh < 0:
            prog.AddConstraint(hand_sliding_constraint_0 >= 0)
            prog.AddConstraint(hand_sliding_constraint_1 == 0)
        else:
            prog.AddConstraint(hand_sliding_constraint_0 >= 0)
            prog.AddConstraint(hand_sliding_constraint_1 >= 0)
        # hand pivoting
        hand_pivoting_constraint_0 = l * (fn+gammas[2]*delta_f_r[1]) - (torque+gammas[2]*delta_torque[0])
        hand_pivoting_constraint_1 = l * (fn+gammas[3]*delta_f_r[1]) + (torque+gammas[3]*delta_torque[0])
        prog.AddConstraint(hand_pivoting_constraint_0 >= 0)
        prog.AddConstraint(hand_pivoting_constraint_1 >= 0)
        # ground sliding
        ground_sliding_constraint_0 = -mu_g * (fj+f_grav_j+gammas[4]*delta_f[1]) + (fi+gammas[4]*delta_f[0])
        ground_sliding_constraint_1 = -mu_g * (fj+f_grav_j+gammas[5]*delta_f[1]) - (fi+gammas[5]*delta_f[0])
        prog.AddConstraint(ground_sliding_constraint_0 >= 0)
        prog.AddConstraint(ground_sliding_constraint_1 >= 0)
        
        # wrench limit
        prog.AddConstraint(fw[0]+delta_f[0] <= wrench_lim[0])
        prog.AddConstraint(fw[1]+delta_f[1] <= wrench_lim[1])
        prog.AddConstraint(torque+delta_torque[0] <= wrench_lim[2])
        prog.AddConstraint(fw[0]+delta_f[0] >= -wrench_lim[0])
        prog.AddConstraint(fw[1]+delta_f[1] >= -wrench_lim[1])
        prog.AddConstraint(torque+delta_torque[0] >= -wrench_lim[2])
        # compliance law
        vars = np.concatenate((delta_f, delta_torque, delta_x_tar))
        """
        delta_w - K @ delta_x_tar == 0
        A_eq = [[1, 0, 0, -K_00, 0, 0]
                [0, 1, 0, 0, -K_11, 0]
                [0, 0, 1, 0, 0, -K_22]]
        """
        R_11 = -np.cos(theta)
        R_12 = np.sin(theta)
        R_21 = -np.sin(theta)
        R_22 = -np.cos(theta)
        A_eq = np.zeros((3, 6))
        # First equation coefficients
        """
        A_eq[0, 0] = R_11       # Coefficient for delta_f[0]
        A_eq[0, 1] = R_12       # Coefficient for delta_f[1]
        """
        A_eq[0, 0] = 1
        A_eq[0, 3] = -K[0, 0]   # Coefficient for delta_x_tar[0]
        # Second equation coefficients
        """
        A_eq[1, 0] = R_21       # Coefficient for delta_f[0]
        A_eq[1, 1] = R_22       # Coefficient for delta_f[1]
        """
        A_eq[1, 1] = 1
        A_eq[1, 4] = -K[1, 1]   # Coefficient for delta_x_tar[1]
        # Third equation: delta_torque[0] - K[2,:] * delta_x_tar = 0
        A_eq[2, 2] = 1  # Coefficient for delta_torque[0]
        A_eq[2, 5] = -K[2, 2]
        # Right-hand side vector
        b_eq = np.zeros(3)
        # Add the linear equality constraint
        prog.AddLinearEqualityConstraint(A_eq, b_eq, vars)
        

        


        solver = OsqpSolver()
        result = solver.Solve(prog)
        optimal_cost = result.get_optimal_cost()
        # print(result.GetSolution(delta_f)[0])
        return result.GetSolution(delta_f), result.GetSolution(delta_torque), result.GetSolution(delta_x_tar), optimal_cost
    
    def world_to_robot_frame(self, input, theta):
        """input: dim=2, can contain symbolic variables."""
        # Ensure input is a list or array of symbolic expressions
        input = [input[0], input[1]]  # Keep as list for compatibility

        # Define the rotation matrix elements using symbolic expressions
        R_11 = -cos(theta)
        R_12 = sin(theta)
        R_21 = -sin(theta)
        R_22 = -cos(theta)

        # Perform the matrix multiplication manually
        output_0 = R_11 * input[0] + R_12 * input[1]
        output_1 = R_21 * input[0] + R_22 * input[1]

        return [output_0, output_1]

    # def solve_QP(self):
    #     return delta_w, delta_x
    
    # def formulate_QP(self, mode, delta):
    #     """
    #     input:
    #         mode: current contact mode
    #         delta: [delta_hand, delta_ground, delta_theta]
    #     """
    #     prog = MathematicalProgram()
    #     if delta[0] != 0:
    #         prog.Add