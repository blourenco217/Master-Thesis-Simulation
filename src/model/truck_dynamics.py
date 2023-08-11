import casadi as ca
import numpy as np

class trailer_unit(object):
    def __init__(self, l, m):
        self.l = l  # length of the segment
        self.m = m  # hitching offset

class vehicle_model(object):
    def __init__(self, segments):
        vehicle = []
        for l, m in segments:
            vehicle.append(trailer_unit(l, m))
        self.vehicle = vehicle

    def dynamics(self, x, u):
        x0_dot = x[2] * ca.cos(x[4])
        y0_dot = x[2] * ca.sin(x[4])
        v0_dot = u[0]
        delta0_dot = u[1]

        phi_dot = []
        phi_dot.append(x[2] * ca.tan(x[3]) / self.vehicle[0].l)
        J_prod = ca.MX.eye(2)
        J = ca.MX(2,2)
        for i in range(1, len(self.vehicle)):
            betha_i = x[4+i-1] - x[4+i]
            J[0,0] = - self.vehicle[i-1].m * ca.cos(betha_i) / self.vehicle[i].l
            J[0,1] = ca.sin(betha_i) / self.vehicle[i].l
            J[1,0] = self.vehicle[i-1].m * ca.sin(betha_i)
            J[1,1] = ca.cos(betha_i)
            J_prod =  J @ J_prod
            phi_dot.append((J_prod @ ca.vertcat(phi_dot[i-1], x[2]))[0])

        
        dxdt = [x0_dot, y0_dot, v0_dot, delta0_dot, *phi_dot]
        return ca.vertcat(*dxdt)