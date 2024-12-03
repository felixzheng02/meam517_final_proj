from pydrake.all import (MathematicalProgram, OsqpSolver)
from pydrake.symbolic import cos, sin, Expression
from pydrake.solvers import SolverOptions
from pydrake.solvers import ScsSolver, SnoptSolver
from pydrake.solvers import MosekSolver
import numpy as np


class Controller:

    def __init__(self):
        pass

    def control(self, theta_tar, sh_tar, d, sh, fw, fr, torque, mu_h, mu_g, theta, l, wrench_lim, K, hand_sliding_constraint_on=True, hand_pivoting_constraint_on=True, ground_sliding_constraint_on=True, w_lim_on=True):

        check_for_nan_inf(theta_tar, sh_tar, d, sh, fw, fr, torque, mu_h, mu_g, theta, l, wrench_lim, K)
        ft, fn = fr
        fi, fj = fw
        fw = np.array([fi, fj])
        wrench_lim = np.array(wrench_lim)
        if type(torque) == np.ndarray:
            torque = torque[0]

        v_theta = np.array([d, 0, 1]).T # hand frame
        v_h = np.array([-1, 0, 0]) # hand frame
        v_g = np.array([1, 0, 0]) # world_frame

        delta_theta = theta_tar - theta
        delta_sh = sh_tar - sh

        prog = MathematicalProgram()
        delta_f = prog.NewContinuousVariables(2, 'delta_f') # [2,] world frame
        delta_f_r = prog.NewContinuousVariables(2, 'delta_f_r') # hand frame
        delta_torque = prog.NewContinuousVariables(1, 'delta_torque')
        delta_x_tar = prog.NewContinuousVariables(3, 'delta_x_tar') # hand frame

        lambda_theta = 90    # Weight for angular deviation (existing)
        lambda_x_tar = 1       
        lambda_f = .1         # Weight for delta_f regularization
        lambda_torque = .01    # Weight for delta_torque regularization
        # Existing cost on angular deviation
        prog.AddQuadraticCost(lambda_theta * (theta + delta_x_tar.T @ v_theta - theta_tar) ** 2 + lambda_x_tar * delta_x_tar.T @ delta_x_tar)
        # New regularization terms to penalize large adjustments
        # prog.AddQuadraticCost(lambda_f_r * delta_f_r.T @ delta_f_r)
        # prog.AddQuadraticCost(lambda_x_tar * delta_x_tar.T @ delta_x_tar)
        # prog.AddQuadraticCost(lambda_torque * delta_torque[0] ** 2)        # prog.AddQuadraticCost((delta_x_tar.T @ v_h - delta_sh) ** 2)

        f_grav_j = -9.81
        gammas = .5 * np.array([1, 1, 1, 1, 1, 1])
        if hand_sliding_constraint_on:
            add_hand_sliding_constraints(prog, mu_h, fn, ft, delta_f_r, gammas)
        if hand_pivoting_constraint_on:
            add_hand_pivoting_constraints(prog, l, fn, torque, delta_f_r, delta_torque, gammas)
        if ground_sliding_constraint_on:
            add_ground_sliding_constraints(prog, mu_g, fi, fj, f_grav_j, delta_f, gammas)
        if w_lim_on:
            add_w_lim_constraints(prog, wrench_lim, fw, torque, delta_f, delta_torque)

        R_11 = -np.cos(theta)
        R_12 = np.sin(theta)
        R_21 = -np.sin(theta)
        R_22 = -np.cos(theta)

        prog.AddLinearEqualityConstraint(np.array([[R_11, R_12, -1, 0], 
                                                   [R_21, R_22, 0, -1]]),
                                         np.zeros(2), 
                                         np.concatenate((delta_f_r, delta_f)))
        
        
        vars = np.concatenate((delta_f_r, delta_torque, delta_x_tar))
        
        # compliance law
        # A_eq = [[1, 0, 0, -K_00, 0, 0]
        #         [0, 1, 0, 0, -K_11, 0]
        #         [0, 0, 1, 0, 0, -K_22]]
        A_eq = np.zeros((3, 6))
        # First equation coefficients
        
        # A_eq[0, 0] = R_11       # Coefficient for delta_f[0]
        # A_eq[0, 1] = R_12       # Coefficient for delta_f[1]
        
        A_eq[0, 0] = 1
        A_eq[0, 3] = -K[0, 0]   # Coefficient for delta_x_tar[0]
        # Second equation coefficients
       
        # A_eq[1, 0] = R_21       # Coefficient for delta_f[0]
        # A_eq[1, 1] = R_22       # Coefficient for delta_f[1]
        
        A_eq[1, 1] = 1
        A_eq[1, 4] = -K[1, 1]   # Coefficient for delta_x_tar[1]
        # Third equation: delta_torque[0] - K[2,:] * delta_x_tar = 0
        A_eq[2, 2] = 1  # Coefficient for delta_torque[0]
        A_eq[2, 5] = -K[2, 2]
        # Right-hand side vector
        b_eq = np.zeros(3)
        # Add the linear equality constraint
        prog.AddLinearEqualityConstraint(A_eq, b_eq, vars)

        # delta_x_tar_range = np.array([.1, .1, .05])
        # for i in range(3):
        #     prog.AddConstraint(delta_x_tar[i] >= -delta_x_tar_range[i])
        #     prog.AddConstraint(delta_x_tar[i] <= delta_x_tar_range[i])

        solver_options = SolverOptions()
        
        solver = SnoptSolver()

        # solver = MosekSolver()
        # solver_options.SetOption(solver.solver_id(), "MSK_IPAR_LOG", 15)  # Set verbosity level
        # solver_options.SetOption(solver.solver_id(), "MSK_IPAR_INFEAS_REPORT_AUTO", 1)  # Automatic infeasibility report
        # solver_options.SetOption(solver.solver_id(), "MSK_IPAR_INFEAS_REPORT_LEVEL", 2)
        # solver_options.get_print_to_console()

        # solver = ScsSolver()
        # solver_options.SetOption(solver.solver_id(), "verbose", True)
        
        result = solver.Solve(prog, None, solver_options)
        if not result.is_success():
            # solver_details = result.get_solver_details()
            # print("Solver status:", solver_details.solution_status)
            infeasible_constraints = result.GetInfeasibleConstraints(prog)
            print("---------------")
            for constraint_binding in infeasible_constraints:
                print(constraint_binding)
                constraint = constraint_binding.evaluator()
                variables = constraint_binding.variables()
                print(f"Constraint type: {type(constraint)}")
                print(f"Involved variables: {variables}")

                # Evaluate the constraint violation
                constraint_value = constraint.Eval(result.GetSolution(variables))
                print(f"Constraint value: {constraint_value}")
                print(f"Lower bound: {constraint.lower_bound()}")
                print(f"Upper bound: {constraint.upper_bound()}")
                print("---------------")
            
            # # Export the model for further analysis
            # prog.WriteMpsFile("model.mps")
            # # Optionally, write the problem in a readable format
            # prog.WriteLpFile("model.lp")
            raise Exception
        
        optimal_cost = result.get_optimal_cost()
        delta_f_sol = result.GetSolution(delta_f)
        delta_torque_sol = result.GetSolution(delta_torque)
        delta_x_tar_sol = result.GetSolution(delta_x_tar)
        if np.any(np.isnan(delta_f_sol)) or np.any(np.isnan(delta_torque_sol)) or np.any(np.isnan(delta_x_tar_sol)):
            print("Nan detected")
            raise Exception
        return delta_f_sol, delta_torque_sol, delta_x_tar_sol, optimal_cost
    
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
    
def add_hand_sliding_constraints(prog, mu_h, fn, ft, delta_f_r, gammas):
    hand_sliding_constraint_0 = mu_h * (fn+gammas[0]*delta_f_r[1]) + (ft+gammas[0]*delta_f_r[0])
    hand_sliding_constraint_1 = mu_h * (fn+gammas[1]*delta_f_r[1]) - (ft+gammas[1]*delta_f_r[0])
    # if delta_sh > 0:
    #     prog.AddConstraint(hand_sliding_constraint_0 == 0)
    #     prog.AddConstraint(hand_sliding_constraint_1 >= 0)
    # elif delta_sh < 0:
    #     prog.AddConstraint(hand_sliding_constraint_0 >= 0)
    #     prog.AddConstraint(hand_sliding_constraint_1 == 0)
    # else:
    prog.AddConstraint(hand_sliding_constraint_0 >= 0)
    prog.AddConstraint(hand_sliding_constraint_1 >= 0)

def add_hand_pivoting_constraints(prog, l, fn, torque, delta_f_r, delta_torque, gammas):
    hand_pivoting_constraint_0 = l * (fn+gammas[2]*delta_f_r[1]) - (torque+gammas[2]*delta_torque[0])
    hand_pivoting_constraint_1 = l * (fn+gammas[3]*delta_f_r[1]) + (torque+gammas[3]*delta_torque[0])
    prog.AddConstraint(hand_pivoting_constraint_0 >= 0)
    prog.AddConstraint(hand_pivoting_constraint_1 >= 0)

def add_ground_sliding_constraints(prog, mu_g, fi, fj, f_grav_j, delta_f, gammas):
    ground_sliding_constraint_0 = -mu_g * (fj+f_grav_j+gammas[4]*delta_f[1]) + (fi+gammas[4]*delta_f[0])
    ground_sliding_constraint_1 = -mu_g * (fj+f_grav_j+gammas[5]*delta_f[1]) - (fi+gammas[5]*delta_f[0])
    prog.AddConstraint(ground_sliding_constraint_0 >= 0)
    prog.AddConstraint(ground_sliding_constraint_1 >= 0)

def add_w_lim_constraints(prog, wrench_lim, fw, torque, delta_f, delta_torque):
    prog.AddConstraint(fw[0]+delta_f[0] <= wrench_lim[0])
    prog.AddConstraint(fw[1]+delta_f[1] <= wrench_lim[1])
    prog.AddConstraint(torque+delta_torque[0] <= wrench_lim[2])
    prog.AddConstraint(fw[0]+delta_f[0] >= -wrench_lim[0])
    prog.AddConstraint(fw[1]+delta_f[1] >= -wrench_lim[1])
    prog.AddConstraint(torque+delta_torque[0] >= -wrench_lim[2])

def check_for_nan_inf(*args):
    for idx, arg in enumerate(args):
        if np.isnan(arg).any() or np.isinf(arg).any():
            print(f"Input argument {idx} contains NaN or Inf values.")
