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
        self.dt = dt                             # step size
        self.N = N                               # prediction horizon
        
        U = ca.MX.sym('U', self.nu, N)           # control input (nu, N)
        X = ca.MX.sym('X', self.nx, N+1)         # state (nx, N+1)
        INITIAL = ca.MX.sym('INITIAL', self.nx)  # initial state (nx, 1)
        REF = ca.MX.sym('REF', self.nx, N)       # reference trajectory (nx, N)

        self.initiate_weights()
        objective = 0                            # cost function
        objective_constrained = 0                # 
        objective_constrained_1 = 0              #
        dynamics_constraints = X[:,0] - INITIAL

        # @obstacle
        if non_convex_constraint or penalty_method:
            obstacle_constraint = []
        # self.obstacle = obstacles()

        for k in range(N):
            x_k = X[:, k]
            u_k = U[:, k]
            ref_x = REF[:, k]
            # objective += 2**k * ((x_k - ref_x).T @ self.Q @ (x_k - ref_x))+ (u_k).T @ self.R @ (u_k) # with exponential weights
            objective += ((x_k - ref_x).T @ self.Q @ (x_k - ref_x))+ (u_k).T @ self.R @ (u_k)


            st_next = X[:, k+1]
            st_next_RK4 = rk4(vehicle.dynamics, x_k[:4 + len(vehicle.vehicle)], u_k[:2], self.dt) 
            dynamics_constraints = ca.vertcat(dynamics_constraints, st_next - st_next_RK4)

            
            # vector_v = ca.vertcat(self.obstacle.center_obstacle[0][0] - self.obstacle.radius_obstacle[0] - x_k[0],
            #                       self.obstacle.center_obstacle[0][1] + self.obstacle.radius_obstacle[0] + vehicle_width/2 - x_k[1])
            # objective_constrained += ((x_k - ref_x).T @ self.Q @ (x_k - ref_x))+ (u_k).T @ self.R @ (u_k) + 4**k * ca.dot(ca.vertcat(INITIAL[0] - x_k[0], INITIAL[1] - x_k[1]), vector_v)

            # TODO
            # compute position last trailer
            x_last = x_k[0]
            y_last = x_k[1]
            for j in range (1, len(self.vehicle.vehicle)):
                theta_prev = x_k[4 + j - 1]
                theta_last = x_k[4 + j]
                x_last -= self.vehicle.vehicle[j].l * ca.cos(theta_last) - self.vehicle.vehicle[j-1].m * ca.cos(theta_prev)
                y_last -= self.vehicle.vehicle[j].l * ca.sin(theta_last) - self.vehicle.vehicle[j-1].m * ca.sin(theta_prev)
            vector_truck = ca.vertcat(x_k[0] - x_last, x_k[1] - y_last)
            # objective_constrained += 2**k * ca.dot(vector_truck, ca.vertcat(INITIAL[0] - x_k[0], INITIAL[1] - x_k[1]))
            
            # TODO
            # if len(self.obstacle.center_obstacle) > 1:
            #     vector_v_1 = ca.vertcat(self.obstacle.center_obstacle[1][0] - self.obstacle.radius_obstacle[1] - x_k[0],
            #                             self.obstacle.center_obstacle[1][1] + self.obstacle.radius_obstacle[1] + vehicle_width/2 - x_k[1])
            #     objective_constrained_1 += ((x_k - ref_x).T @ self.Q @ (x_k - ref_x))+ (u_k).T @ self.R @ (u_k) + 4**k * ca.dot(ca.vertcat(INITIAL[0] - x_k[0], INITIAL[1] - x_k[1]), vector_v_1)
                

            if non_convex_constraint: # non-convex constraint for obstacle avoidance
                x_n = x_k[0]
                y_n = x_k[1]
                theta_n = x_k[2]
                pos = ca.vertcat(x_n, y_n)
                for j in range(len(self.obstacle.center_obstacle)):
                    h = (pos - self.obstacle.center_obstacle[j]).T @ (pos - self.obstacle.center_obstacle[j]) - self.obstacle.radius_obstacle[j]**2
                    obstacle_constraint = ca.vertcat(obstacle_constraint, h)
                    # ca.dot()
            elif penalty_method: # penalty method for obstacle avoidance
                # loop over the different obstacles
                x_n = x_k[0]
                y_n = x_k[1]
                theta_n = x_k[2]
                pos = ca.vertcat(x_n, y_n)
                for j in range(len(self.obstacle.center_obstacle)):
                    dist = ca.norm_2(pos - self.obstacle.center_obstacle[j]) - self.obstacle.radius_obstacle[j]
                    objective += ca.if_else(dist > 0, 1000 * (k + 1) * dist, 0)
                    # ref_x[1,k] = ca.if_else(dist > 0, ref_x[1,k] + 1.75, ref_x[1,k])
                # # dist = (pos - barrier.center_obstacle).T @ (pos - barrier.center_obstacle) - barrier.radius_obstacle**2
                # dist = ca.norm_2(pos - barrier.center_obstacle) - barrier.radius_obstacle
                # objective += ca.if_else(dist > 0, 1000 * (k + 1) * dist, 0)
        
        opt_variables = ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1)))
        if non_convex_constraint:
            constraints = ca.vertcat(dynamics_constraints, obstacle_constraint)
        else:
            constraints = ca.vertcat(dynamics_constraints)
        P = ca.vertcat(INITIAL, REF.reshape((-1, 1)))


        nlp = {
            'f': objective,
            'x': opt_variables,
            'g': constraints,
            'p': P
        }

        nlp_constrained = {
            'f': objective_constrained,
            'x': opt_variables,
            'g': constraints,
            'p': P
        }

        # if len(self.obstacle.center_obstacle) > 1:
        #     nlp_constrained_1 = {
        #         'f': objective_constrained_1,
        #         'x': opt_variables,
        #         'g': constraints,
        #         'p': P
        #     }
        self.solver_unconstrained = ca.nlpsol('solver', 'ipopt', nlp, MPC_OPTS)
        self.solver_constrained = ca.nlpsol('solver', 'ipopt', nlp_constrained, MPC_OPTS)
        # if len(self.obstacle.center_obstacle) > 1:
        #     self.solver_constrained_1 = ca.nlpsol('solver', 'ipopt', nlp_constrained_1, MPC_OPTS)
        print(self.solver_unconstrained.stats())
        self.initiate_constraints()
  
    def initiate_weights(self):
        weights = [100, 10, 10, 10, 100]
        for _ in range(1,len(self.vehicle.vehicle)):
            weights.append(100)
        self.Q = ca.diagcat(*weights)      # state weights matrix
        self.R = ca.diagcat(10, 10)      # control weights matrix

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

        lbx[self.nx*(self.N+1):self.nx*(self.N+1) +self.nu*self.N:self.nu] =  -ca.inf      # lower bound for steering
        ubx[self.nx*(self.N+1):self.nx*(self.N+1) +self.nu*self.N:self.nu] = ca.inf        # upper bound for steering

        lbx[self.nx*(self.N+1)+1 :self.nx*(self.N+1) +self.nu*self.N :self.nu] = -ca.inf   # lower bound for throttle    
        ubx[self.nx*(self.N+1)+1 :self.nx*(self.N+1) +self.nu*self.N :self.nu] = ca.inf    # upper bound for throttle


        if non_convex_constraint:
            lbg = ca.DM.zeros((self.nx*(self.N+1) + (int(len(self.obstacle.center_obstacle))) * self.N, 1))
            ubg = ca.DM.zeros((self.nx*(self.N+1) + (int(len(self.obstacle.center_obstacle))) * self.N, 1))
            lbg[-self.N:] = 0
            ubg[-self.N:] = ca.inf
        else:
            lbg = ca.DM.zeros((self.nx*(self.N+1), 1))  # constraints lower bound
            ubg = ca.DM.zeros((self.nx*(self.N+1), 1))  # constraints upper bound
        self.args = {
            'lbg': lbg,
            'ubg': ubg,
            'lbx': lbx,
            'ubx': ubx,
            'p': 0
        }


    def is_within_obstacle(self, X0):
        # detect obstacle in horizon

        for i in range(len(self.obstacle.center_obstacle)):
            for j in range(self.N):
                pos = ca.vertcat(X0[0,j], X0[1,j])
                dist = ca.norm_2(pos - self.obstacle.center_obstacle[i])
                if dist < self.obstacle.radius_obstacle[i]:
                    return True, i
        return False
    
    def has_overtaken_obstacle(self, X0, inx):
        # detect obstacle in horizon
        for i in range(len(self.obstacle.center_obstacle)):
            if X0[0,0] >= self.obstacle.center_obstacle[inx][0]:
                return True, i
        return False, i
