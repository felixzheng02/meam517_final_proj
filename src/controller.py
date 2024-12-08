from pydrake.all import (MathematicalProgram, OsqpSolver)
from pydrake.symbolic import cos, sin, Expression
from pydrake.solvers import SolverOptions
from pydrake.solvers import ScsSolver, SnoptSolver
from pydrake.solvers import MosekSolver
import numpy as np


class Controller:

    def __init__(self, l, w_lim=100*np.identity(3), K=np.identity(3), lambda_theta=100, lambda_x_tar=1, 
                 hand_sliding_constraint_on=True, hand_pivoting_constraint_on=True, ground_sliding_constraint_on=True, w_lim_on=True, delta_x_tar_lim_on=True, momentum_control_on=False):
        self.lambda_theta = lambda_theta
        self.lambda_x_tar = lambda_x_tar
        self.l = l
        self.w_lim = w_lim
        self.K = K
        self.hand_sliding_constraint_on = hand_sliding_constraint_on
        self.hand_pivoting_constraint_on = hand_pivoting_constraint_on
        self.ground_sliding_constraint_on = ground_sliding_constraint_on
        self.w_lim_on = w_lim_on
        self.delta_x_tar_lim_on = delta_x_tar_lim_on
        self.momentum_control_on = momentum_control_on
        self.last_delta_theta = 0


    def control(self, theta_tar, sh_tar, d, sh, fw, fr, torque, mu_h, mu_g, theta):
        check_for_nan_inf(theta_tar, sh_tar, d, sh, fw, fr, torque, mu_h, mu_g, theta)
        ft, fn = fr
        fi, fj = fw
        fw = np.array([fi, fj])
        w_lim = np.array(self.w_lim)
        if type(torque) == np.ndarray:
            torque = torque[0]

        v_theta = np.array([d, -sh, 1]).T # hand frame
        v_h = np.array([-1, 0, 0]) # hand frame
        v_g = np.array([1, 0, 0]) # world_frame

        delta_theta = theta_tar - theta
        delta_sh = sh_tar - sh

        prog = MathematicalProgram()
        delta_f = prog.NewContinuousVariables(2, 'delta_f') # [2,] world frame
        delta_f_r = prog.NewContinuousVariables(2, 'delta_f_r') # hand frame
        delta_torque = prog.NewContinuousVariables(1, 'delta_torque')
        delta_x_tar = prog.NewContinuousVariables(3, 'delta_x_tar') # hand frame     
        
        prog.AddQuadraticCost(self.lambda_theta * (-delta_theta*v_theta + delta_x_tar).T @ (-delta_theta*v_theta + delta_x_tar)
                              + self.lambda_x_tar * delta_x_tar.T @ delta_x_tar)
                            #   + 0 * delta_f_r.T @ delta_f_r
                            #   + 0 * delta_torque ** 2)

        f_grav_j = -9.81
        gammas = 1 * np.array([1, 1, 1, 1, 1, 1])
        if self.hand_sliding_constraint_on:
            add_hand_sliding_constraints(prog, mu_h, fn, ft, delta_f_r, gammas)
        if self.hand_pivoting_constraint_on:
            add_hand_pivoting_constraints(prog, self.l, fn, torque, delta_f_r, delta_torque, gammas)
        if self.ground_sliding_constraint_on:
            add_ground_sliding_constraints(prog, mu_g, fi, fj, f_grav_j, delta_f, gammas)
        if self.w_lim_on:
            add_w_lim_constraints(prog, w_lim, fw, torque, delta_f, delta_torque)

        prog.AddLinearEqualityConstraint(np.array([[-np.cos(theta), np.sin(theta), -1, 0], 
                                                   [-np.sin(theta), -np.cos(theta), 0, -1]]),
                                         np.zeros(2), 
                                         np.concatenate((delta_f_r, delta_f)))
        
        # compliance law
        # A_eq = [[1, 0, 0, -K_00, 0, 0]
        #         [0, 1, 0, 0, -K_11, 0]
        #         [0, 0, 1, 0, 0, -K_22]]
        A_eq = np.zeros((3, 6))       
        A_eq[0, 0] = 1
        A_eq[0, 3] = -self.K[0, 0] 
        A_eq[1, 1] = 1
        A_eq[1, 4] = -self.K[1, 1]
        A_eq[2, 2] = 1
        A_eq[2, 5] = -self.K[2, 2]
        b_eq = np.zeros(3)
        prog.AddLinearEqualityConstraint(A_eq, b_eq, np.concatenate((delta_f_r, delta_torque, delta_x_tar)))

        if self.delta_x_tar_lim_on:
            delta_x_tar_range = np.array([.1, .1, .01])
            # if abs(delta_theta) < np.pi/6:
            #     delta_x_tar_range = .5 * delta_x_tar_range
            for i in range(3):
                prog.AddConstraint(delta_x_tar[i] >= -delta_x_tar_range[i])
                prog.AddConstraint(delta_x_tar[i] <= delta_x_tar_range[i])

        solver_options = SolverOptions()
        # solver = SnoptSolver()

        # solver = MosekSolver()
        # solver_options.SetOption(solver.solver_id(), "MSK_IPAR_LOG", 15)  # Set verbosity level
        # solver_options.SetOption(solver.solver_id(), "MSK_IPAR_INFEAS_REPORT_AUTO", 1)  # Automatic infeasibility report
        # solver_options.SetOption(solver.solver_id(), "MSK_IPAR_INFEAS_REPORT_LEVEL", 2)
        # solver_options.get_print_to_console()

        solver = ScsSolver()
        solver_options.SetOption(solver.solver_id(), "verbose", False)
        
        result = solver.Solve(prog, None, solver_options)
        if not result.is_success():
            solver_details = result.get_solver_details()
            solution_status = result.get_solution_result()
            print("Solver status:", solution_status)
            # infeasible_constraints = result.GetInfeasibleConstraints(prog)
            # print("---------------")
            # for constraint_binding in infeasible_constraints:
            #     print(constraint_binding)
            #     constraint = constraint_binding.evaluator()
            #     variables = constraint_binding.variables()
            #     print(f"Constraint type: {type(constraint)}")
            #     print(f"Involved variables: {variables}")

            #     # Evaluate the constraint violation
            #     constraint_value = constraint.Eval(result.GetSolution(variables))
            #     print(f"Constraint value: {constraint_value}")
            #     print(f"Lower bound: {constraint.lower_bound()}")
            #     print(f"Upper bound: {constraint.upper_bound()}")
            #     print("---------------")
            raise Exception
        
        optimal_cost = result.get_optimal_cost()
        delta_f_sol = result.GetSolution(delta_f)
        delta_f_r_sol = result.GetSolution(delta_f_r)
        delta_torque_sol = result.GetSolution(delta_torque)
        delta_x_tar_sol = result.GetSolution(delta_x_tar)
        if np.any(np.isnan(delta_f_sol)) or np.any(np.isnan(delta_torque_sol)) or np.any(np.isnan(delta_x_tar_sol)):
            print("Nan detected")
            raise Exception
        if self.momentum_control_on:
            delta_f_sol, delta_torque_sol = self.momentum_control(delta_f_r_sol, delta_theta, mu_h, mu_g, theta, ft, fn, fi, fj, torque, w_lim)
        self.last_delta_theta = delta_theta
        return delta_f_sol, delta_torque_sol, delta_x_tar_sol, optimal_cost

    def momentum_control(self, delta_f_r_sol, delta_theta, mu_h, mu_g, theta, ft, fn, fi, fj, torque, w_lim):
        delta_ft = delta_f_r_sol[0]
        if abs(delta_theta) - abs(self.last_delta_theta) > 0: # moving away from target
            delta_ft_tar = 2 * delta_ft
            if ft*delta_theta < 0: # force not overturned to correct direction
                delta_ft_tar = 3 * delta_ft_tar
        else: # moving towards target
            if delta_ft*delta_theta > .1:
                delta_ft_tar = .5 * delta_ft
            else: # == 0, never < 0
                delta_ft_tar = -10
                
                

        prog = MathematicalProgram()
        delta_f_adjusted = prog.NewContinuousVariables(2, 'delta_f_adjusted')
        delta_f_r_adjusted = prog.NewContinuousVariables(2, 'delta_f_r_adjusted') # hand frame
        delta_torque_adjusted = prog.NewContinuousVariables(1, 'delta_torque_adjusted')    
        
        lambda_f = 100
        lambda_torque = 1
        prog.AddQuadraticCost(lambda_f * (delta_f_r_adjusted[0] - delta_ft_tar)**2
                              + lambda_torque * delta_torque_adjusted[0] ** 2)

        f_grav_j = -9.81
        gammas = 1 * np.array([1, 1, 1, 1, 1, 1])
        if self.hand_sliding_constraint_on:
            add_hand_sliding_constraints(prog, mu_h, fn, ft, delta_f_r_adjusted, gammas)
        if self.hand_pivoting_constraint_on:
            add_hand_pivoting_constraints(prog, self.l, fn, torque, delta_f_r_adjusted, delta_torque_adjusted, gammas)
        if self.ground_sliding_constraint_on:
            add_ground_sliding_constraints(prog, mu_g, fi, fj, f_grav_j, delta_f_adjusted, gammas)
        if self.w_lim_on:
            add_w_lim_constraints(prog, w_lim, np.array([fi, fj]), torque, delta_f_adjusted, delta_torque_adjusted)

        prog.AddLinearEqualityConstraint(np.array([[-np.cos(theta), np.sin(theta), -1, 0], 
                                                   [-np.sin(theta), -np.cos(theta), 0, -1]]),
                                         np.zeros(2), 
                                         np.concatenate((delta_f_r_adjusted, delta_f_adjusted)))
        
        solver_options = SolverOptions()
        solver = ScsSolver()
        solver_options.SetOption(solver.solver_id(), "verbose", False)
        
        result = solver.Solve(prog, None, solver_options)
        if not result.is_success():
            solver_details = result.get_solver_details()
            solution_status = result.get_solution_result()
            print("Solver status:", solution_status)
            raise Exception
        
        optimal_cost = result.get_optimal_cost()
        delta_f_sol = result.GetSolution(delta_f_adjusted)
        delta_f_r_sol = result.GetSolution(delta_f_r_adjusted)
        delta_torque_sol = result.GetSolution(delta_torque_adjusted)
        if np.any(np.isnan(delta_f_sol)) or np.any(np.isnan(delta_torque_sol)) or np.any(np.isnan(delta_f_r_sol)):
            print("Nan detected")
            raise Exception
        return delta_f_sol, delta_torque_sol
        

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
