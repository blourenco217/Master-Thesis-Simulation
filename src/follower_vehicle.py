#!/usr/bin/env python

import rospy
import casadi as ca
import numpy as np
from model.truck_dynamics import vehicle_model
from controller.mpc import mpc
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class FollowerCACCNode:
    def __init__(self):
        rospy.init_node('follower_cacc_node', anonymous=True)
        
        # Parameters
        self.target_headway = 1.0  # Desired time gap between vehicles (in seconds)
        self.kp = 0.5              # Proportional gain
        
        # Subscribers
        rospy.Subscriber('ego_vehicle/odom', Odometry, self.odom_ego_callback)
        rospy.Subscriber('ego_vehicle/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/follower_vehicle/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/follower_vehicle/odom', Odometry, self.odometry_follower_callback)
        
        # Initializations
        self.ego_pose = (0.0, 0.0, 0.0)
        self.ego_vel = [0.0,0.0,0.0]
        self.ego_twist = Twist()
        self.follower_twist = Twist()



        self.follower_pose = (0.0, 0.0, 0.0)
        self.rate = rospy.Rate(10) # 10 Hz
        dt = 1/10
        segments = [(3.5, 0.0),(6.0, 0.0)]
        vehicle = vehicle_model(segments)
        self.controller = mpc(vehicle, dt, N = 12)
        self.u0 = ca.DM.zeros((self.controller.nu, self.controller.N))
        self.x0 = ca.DM.zeros((self.controller.nx))


        self.hitch_angle_pub = rospy.Publisher('/follower_vehicle/hitch_joint_position_controller/command', Float64, queue_size=10)
        self.hitch_angle_sub = rospy.Subscriber('/follower_vehicle/hitch_joint_position_controller/state', Float64, self.hitch_angle_callback)
        self.hitch_angle = 0

    def hitch_angle_callback(self, msg):
        self.hitch_angle = msg.process_value

    def odometry_follower_callback(self, odom_msg):
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        self.follower_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])
        # self.follower_vel = (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z)
        
    def odom_ego_callback(self, odom_msg):
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        self.ego_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])
        self.ego_vel = (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z)
        
    def cmd_vel_callback(self, cmd_vel_msg):
        self.ego_twist = cmd_vel_msg

        
    def control_loop(self):
        
        while not rospy.is_shutdown():
            X0 = ca.repmat(self.x0, 1, self.controller.N+1)
            self.controller.args['x0'] = [self.follower_pose[0], self.follower_pose[1], 0, 0, self.follower_pose[2], 0]
            self.cmd_vel_pub.publish(self.follower_twist)
            self.controller.args['p'] = ca.vertcat(self.controller.args['x0'])

            for i in range(self.controller.N):
                # self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.ego_pose[0], self.ego_pose[1], 
                #                                        np.sqrt(self.ego_vel[0]**2 + self.ego_vel[1]**2), self.ego_vel[2],
                #                                          self.ego_pose[2], 0)

                x_ref = self.ego_pose[0] + self.ego_vel[0] * self.controller.dt * (i+1)
                y_ref = self.ego_pose[1] + self.ego_vel[1] * self.controller.dt * (i+1)
                # x_ref = 200
                # y_ref = 7.5
                self.controller.args['p'] = ca.vertcat(self.controller.args['p'], x_ref, y_ref, 
                                                       0, 0, 0, 0)

            self.controller.args['x0'] = ca.vertcat(
                ca.reshape(X0, self.controller.nx*(self.controller.N+1), 1),
                ca.reshape(self.u0, self.controller.nu*self.controller.N, 1)
            )

            sol = self.controller.solver_unconstrained(
                        x0 = self.controller.args['x0'],
                        lbx = self.controller.args['lbx'],
                        ubx = self.controller.args['ubx'],
                        lbg = self.controller.args['lbg'],
                        ubg = self.controller.args['ubg'],
                        p = self.controller.args['p']
                    )
            # print(self.controller.solver_unconstrained.stats()['return_status'])
            u = ca.reshape(sol['x'][self.controller.nx * (self.controller.N + 1):], self.controller.nu, self.controller.N)
            X0 = ca.reshape(sol['x'][: self.controller.nx * (self.controller.N+1)], self.controller.nx, self.controller.N+1)
            
            self.u0 = u
            # print(X0[0,1], X0[1,1])
            self.follower_twist.linear.x = X0[2,1]
            self.follower_twist.angular.z = X0[3,1]
            self.x0 = X0[:,1]

            self.x0[0] = self.follower_pose[0]
            self.x0[1] = self.follower_pose[1]
            self.x0[2] = self.follower_pose[2]
            print(self.follower_pose)
            self.cmd_vel_pub.publish(self.follower_twist)
            command = Float64()
            command.data = X0[4, 1] - X0[5, 1]
            self.hitch_angle_pub.publish(command)
            self.rate.sleep()



            # # Calculate desired follower velocity based on CACC algorithm
            # desired_follower_vel_x = self.ego_vel[0] + self.kp * (self.target_headway - self.ego_twist.linear.x)
            # desired_follower_vel_y = self.ego_vel[1] + self.kp * (self.target_headway - self.ego_twist.linear.y)
            # desired_follower_rot_z = self.ego_vel[2] + self.kp * (self.target_headway - self.ego_twist.angular.z)
            
            # # Create a new Twist message with the desired follower velocity
            # self.follower_twist.linear.x = desired_follower_vel_x
            # # self.follower_twist.linear.y = desired_follower_vel_y
            # self.follower_twist.angular.z = desired_follower_rot_z
            
            # # Publish the new twist command for the follower vehicle
            # self.cmd_vel_pub.publish(self.follower_twist)
            
            # self.rate.sleep()

if __name__ == '__main__':
    try:
        follower_node = FollowerCACCNode()
        follower_node.control_loop()
    except rospy.ROSInterruptException:
        pass
