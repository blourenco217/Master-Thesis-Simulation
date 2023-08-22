import casadi as ca
import numpy as np
from controller.utils import *
from time import time
# from obstacle.obstacle import obstacles
import math

noise = False
process_noise_standard_deviations = 0.01
non_convex_constraint = False
penalty_method = False

verbose = False

vehicle_width = 3

class mpc(object):
    def __init__(self, vehicle, dt = 0.1, N = 10):
        self.vehicle = vehicle
        self.nx = len(vehicle.vehicle) + 4
        self.nu = 2
        self.dt = dt                                     # step size
        self.N = N                                       # prediction horizon

        
        U = ca.MX.sym('U', self.nu, N)                   # control input (nu, N)
        X = ca.MX.sym('X', self.nx, N+1)                 # state (nx, N+1)
        INITIAL = ca.MX.sym('INITIAL', self.nx)          # initial state (nx, 1)
        REF = ca.MX.sym('REF', self.nx, N)               # reference trajectory (nx, N)

        OBSTACLE_POS = ca.MX.sym('OBSTACLE_POS', 2)      # obstacle current position (2, 1)
        OBSTACLE_VEL = ca.MX.sym('OBSTACLE_VEL', 2)      # obstacle current velocity (2, 1)

        self.initiate_weights()
        objective = 0                            # cost function
        objective_constrained = 0 
        dynamics_constraints = X[:,0] - INITIAL


        for k in range(N):
            x_k = X[:, k]
            u_k = U[:, k]
            ref_x = REF[:, k]
            objective += ((x_k - ref_x).T @ self.Q @ (x_k - ref_x))+ (u_k).T @ self.R @ (u_k)
            st_next = X[:, k+1]
            st_next_RK4 = rk4(vehicle.dynamics, x_k[:4 + len(vehicle.vehicle)], u_k[:2], self.dt) 
            dynamics_constraints = ca.vertcat(dynamics_constraints, st_next - st_next_RK4)

            # prediction_obstacle_position = (OBSTACLE_POS[0] + k * OBSTACLE_VEL[0] * self.dt, OBSTACLE_POS[1] + k * OBSTACLE_VEL[1] * self.dt)
            prediction_obstacle_position = (OBSTACLE_POS[0] + k * 2 * self.dt, OBSTACLE_POS[1])
            vector_to_obstacle = ca.vertcat(prediction_obstacle_position[0] - x_k[0], prediction_obstacle_position[1] + 1.5 - x_k[1])
            vector_velocity = ca.vertcat(INITIAL[0] - x_k[0], INITIAL[1] - x_k[1])
            objective_constrained += ((x_k - ref_x).T  @ (x_k - ref_x))+ (u_k).T  @ (u_k) + \
                     3.4**(k+1.35) * ca.dot(vector_velocity, vector_to_obstacle)

        
        opt_variables = ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1)))
        constraints = ca.vertcat(dynamics_constraints)
        P = ca.vertcat(INITIAL, REF.reshape((-1, 1)))


        nlp = {
            'f': objective,
            'x': opt_variables,
            'g': constraints,
            'p': P
        }

        P_constrained = ca.vertcat(INITIAL, REF.reshape((-1, 1)), OBSTACLE_POS.reshape((-1, 1)), OBSTACLE_VEL.reshape((-1, 1)))

        nlp_constrained = {
            'f': objective_constrained,
            'x': opt_variables,
            'g': constraints,
            'p': P_constrained
        }
    
        self.solver_unconstrained = ca.nlpsol('solver', 'ipopt', nlp, MPC_OPTS)
        self.solver_constrained = ca.nlpsol('solver', 'ipopt', nlp_constrained, MPC_OPTS)
        if verbose:
            print(self.solver_unconstrained.stats())
            print(self.solver_constrained.stats())
        self.initiate_constraints()
  
    def initiate_weights(self):
        weights = [100, 100, 10, 0, 10]
        for _ in range(1,len(self.vehicle.vehicle)):
            weights.append(0)
        self.Q = ca.diagcat(*weights)      # state weights matrix
        self.R = ca.diagcat(5, 5)      # control weights matrix

    def initiate_constraints(self):
        lbx = ca.DM.zeros((self.nx*(self.N+1) +self.nu*self.N, 1))
        ubx = ca.DM.zeros((self.nx*(self.N+1) +self.nu*self.N, 1))
        
        # lower bounds
        lbx[0: self.nx*(self.N+1):self.nx] = -ca.inf        # x0 lower bound
        lbx[1: self.nx*(self.N+1):self.nx] = -ca.inf        # y0 lower bound
        lbx[2: self.nx*(self.N+1):self.nx] = -ca.inf        # v0 bound
        lbx[3: self.nx*(self.N+1):self.nx] = -ca.pi/3       # delta0 lower bound
        for i in range(4, 4 + len(self.vehicle.vehicle)):
            lbx[i:self.nx*(self.N+1):self.nx] = -ca.inf     # betha_i lower bound

        ubx[0:self.nx*(self.N+1):self.nx] = ca.inf          # x0 upper bound
        ubx[1:self.nx*(self.N+1):self.nx] = ca.inf          # y0 upper bound
        ubx[2:self.nx*(self.N+1):self.nx] = ca.inf          # v0 upper bound
        ubx[3:self.nx*(self.N+1):self.nx] = ca.pi/3         # delta0 upper bound
        for i in range(4, 4 + len(self.vehicle.vehicle)):   
            ubx[i:self.nx*(self.N+1):self.nx] = ca.inf      # betha_i upper bound

        lbx[self.nx*(self.N+1):self.nx*(self.N+1) +self.nu*self.N:self.nu] =  -5      # lower bound for steering
        ubx[self.nx*(self.N+1):self.nx*(self.N+1) +self.nu*self.N:self.nu] = 100             # upper bound for steering

        lbx[self.nx*(self.N+1)+1 :self.nx*(self.N+1) +self.nu*self.N :self.nu] = -100   # lower bound for throttle    
        ubx[self.nx*(self.N+1)+1 :self.nx*(self.N+1) +self.nu*self.N :self.nu] = 100    # upper bound for throttle


        lbg = ca.DM.zeros((self.nx*(self.N+1), 1))  # constraints lower bound
        ubg = ca.DM.zeros((self.nx*(self.N+1), 1))  # constraints upper bound
        self.args = {
            'lbg': lbg,
            'ubg': ubg,
            'lbx': lbx,
            'ubx': ubx
        }
