#!/usr/bin/env python

from model.truck_dynamics import vehicle_model
from controller.mpc import mpc
from avoidance.obstacle_extraction import ObstacleExtraction

import rospy
import casadi as ca
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan



class EgoVehicleController(object):
    def __init__(self, vehicle):
        
        self.obstacle_extraction = ObstacleExtraction()
        self.vehicle = vehicle
        rospy.init_node('ego_vehicle_controller', anonymous=True)
        self.scan_sub = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/ego_vehicle/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/ego_vehicle/odom', Odometry, self.odometry_callback)
        self.hitch_angle_pub = rospy.Publisher('/ego_vehicle/hitch_joint_position_controller/command', Float64, queue_size=10)
        self.hitch_angle_sub = rospy.Subscriber('/ego_vehicle/hitch_joint_position_controller/state', Float64, self.hitch_angle_callback)

        self.rate = rospy.Rate(10) # 10 Hz

        dt = 1/10
        self.controller = mpc(self.vehicle, dt, N = 10)

        self.twist_cmd = Twist()

        self.position = (0.0, 0.0)  # Initialize the position
        self.hitch_angle = 0

        self.reference = self.position[0]

        rospy.loginfo("Ego-Vehicle Controller Initialized.")

    def scan_callback(self, msg):
        self.obstacle_extraction.scan_callback(msg)

        if self.obstacle_extraction.obstacle_ahead:
            pass
    
    def odometry_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def hitch_angle_callback(self, msg):
        self.hitch_angle = msg.process_value


    def move(self):
        while not rospy.is_shutdown():
            # self.twist_cmd.linear.x, self.twist_cmd.angular.z = self.controller.control(self.position)
            u0 = ca.DM.zeros((self.controller.nu, self.controller.N))
            x0 = ca.DM.zeros((self.controller.nx))

            self.controller.args['x0'] = [self.position[0], self.position[1], 0, 0, 0, 0]
            self.cmd_vel_pub.publish(self.twist_cmd)
            self.controller.args['p'] = ca.vertcat(self.controller.args['x0'])

            X0 = ca.repmat(x0, 1, self.controller.N+1)

            self.reference += 1
            for i in range(self.controller.N):
                t_predict = self.reference + i * self.controller.dt 
                x_ref = 200
                # x_ref = self.reference
                
                y_ref = 3.5
                # if mpc_iter > 200:
                #     y_ref = 1.75
                theta_ref = 0
                betha_1_ref = 0
                betha_2_ref = 0
                self.controller.args['p'] = ca.vertcat(self.controller.args['p'], x_ref, y_ref, theta_ref)
                for _ in range(1, 2 + len(self.vehicle.vehicle)):
                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], 0)

            # optimization variable current state
            self.controller.args['x0'] = ca.vertcat(
                ca.reshape(X0, self.controller.nx*(self.controller.N+1), 1),
                ca.reshape(u0, self.controller.nu*self.controller.N, 1)
            )
            if self.obstacle_extraction.obstacle_ahead:
                leftmost_boundary = np.array(self.obstacle_extraction.leftmost_boundary).reshape(2,)
                predicted_velocity = np.array(self.obstacle_extraction.predicted_velocity).reshape(2,)
                self.controller.args['p'] = ca.vertcat(self.controller.args['p'], leftmost_boundary[0], leftmost_boundary[1])
                self.controller.args['p'] = ca.vertcat(self.controller.args['p'], predicted_velocity[0], predicted_velocity[1])
                
                # self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.obstacle_extraction.leftmost_boundary)
                # self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.obstacle_extraction.leftmost_boundary[0], self.obstacle_extraction.leftmost_boundary[1])
                
                # self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.obstacle_extraction.predicted_velocity[0], self.obstacle_extraction.predicted_velocity[1])
                sol = self.controller.solver_constrained(
                    x0 = self.controller.args['x0'],
                        lbx = self.controller.args['lbx'],
                        ubx = self.controller.args['ubx'],
                        lbg = self.controller.args['lbg'],
                        ubg = self.controller.args['ubg'],
                        p = self.controller.args['p']
                    )
            else:
                sol = self.controller.solver_unconstrained(
                        x0 = self.controller.args['x0'],
                        lbx = self.controller.args['lbx'],
                        ubx = self.controller.args['ubx'],
                        lbg = self.controller.args['lbg'],
                        ubg = self.controller.args['ubg'],
                        p = self.controller.args['p']
                    )
            u = ca.reshape(sol['x'][self.controller.nx * (self.controller.N + 1):], self.controller.nu, self.controller.N)
            X0 = ca.reshape(sol['x'][: self.controller.nx * (self.controller.N+1)], self.controller.nx, self.controller.N+1)

            u0 = u
            self.twist_cmd.linear.x = u[0, 0]
            # self.twist_cmd.linear.x = u[0, 0] * ca.cos(self.controller.args['x0'][4])
            # self.twist_cmd.linear.y =  u[1, 0] * ca.sin(self.controller.args['x0'][4])
            self.twist_cmd.angular.z = u[1, 0]
            # self.twist_cmd.linear.x = self.controller.args['x0'][0]
            # self.twist_cmd.linear.y = self.controller.args['x0'][1]
            # self.twist_cmd.angular.z = self.controller.args['x0'][4]


            self.cmd_vel_pub.publish(self.twist_cmd)
            command = Float64()
            # command.data = self.controller.args['x0'][2]
            command.data = self.controller.args['x0'][4] - self.controller.args['x0'][5]
            self.hitch_angle_pub.publish(command)

            self.rate.sleep()


class MoveAndPrintPosition:
    def __init__(self):
        rospy.init_node('move_and_print_position', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 3.0   # Set the desired linear velocity
        self.twist_cmd.angular.z = 0.0  # No angular velocity

        self.position = (0.0, 0.0)  # Initialize the position

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def move_straight(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist_cmd)
            rospy.loginfo("Current position: x = {:.2f}, y = {:.2f}".format(*self.position))
            self.rate.sleep()

def control_hitch_joint():
    rospy.init_node('hitch_joint_controller', anonymous=True)
    pub = rospy.Publisher('/hitch_joint_position_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        command = Float64()
        command.data = 1
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    segments = [(3.5, 0.0),(6.0, 0.0)]
    vehicle = vehicle_model(segments)
    try:
        node = EgoVehicleController(vehicle)
        node.move()

        # move_print = MoveAndPrintPosition()
        # move_print.move_straight()
        # control_hitch_joint()
    except rospy.ROSInterruptException:
        pass