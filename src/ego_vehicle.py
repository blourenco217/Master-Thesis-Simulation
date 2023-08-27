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
from tf.transformations import euler_from_quaternion
from control_msgs.msg import JointControllerState



class EgoVehicleController(object):
    def __init__(self, vehicle):
        
        self.obstacle_extraction = ObstacleExtraction()
        self.vehicle = vehicle
        rospy.init_node('ego_vehicle_controller', anonymous=True)
        self.scan_sub = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/ego_vehicle/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/ego_vehicle/odom', Odometry, self.odometry_callback)
        self.hitch_angle_pub = rospy.Publisher('/ego_vehicle/hitch_joint_position_controller/command', Float64, queue_size=10)
        self.hitch_angle_sub = rospy.Subscriber('/ego_vehicle/hitch_joint_position_controller/state', JointControllerState, self.hitch_angle_callback)

        self.rate = rospy.Rate(10) # 10 Hz

        dt = 1/10
        self.controller = mpc(self.vehicle, dt, N = 10)
        self.u0 = ca.DM.zeros((self.controller.nu, self.controller.N))
        self.x0 = ca.DM.zeros((self.controller.nx))

        self.twist_cmd = Twist()

        self.ego_pose = (0.0, 0.0, 0.0)  # Initialize the position
        self.hitch_angle = 0
        self.overtaken = True # if overtaken is True, then the ego vehicle is behind the obstacle

        self.save_data = False
        self.state_array = []
        self.input_array = []
        self.time_array = []
        rospy.loginfo("Ego-Vehicle Controller Initialized.")

    def scan_callback(self, msg):
        self.obstacle_extraction.scan_callback(msg)

        # if self.obstacle_extraction.obstacle_ahead:
        #     pass
    
    def odometry_callback(self, msg):
        # self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.ego_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])
        self.obstacle_extraction.ego_pose = self.ego_pose
    
    def hitch_angle_callback(self, msg):
        self.hitch_angle = msg.process_value


    def move(self):

        try:
        
            while not rospy.is_shutdown():
                # self.twist_cmd.linear.x, self.twist_cmd.angular.z = self.controller.control(self.position)
                X0 = ca.repmat(self.x0, 1, self.controller.N+1)

                # self.controller.args['x0'] = [self.ego_pose[0], self.ego_pose[1], X0[2, 1], X0[3, 1], self.ego_pose[2], 0]
                self.controller.args['x0'] = [self.ego_pose[0], self.ego_pose[1], float(self.x0[2]), float(self.x0[3]), self.ego_pose[2], float(self.x0[5])]

                # self.controller.args['x0'] = [self.ego_pose[0], self.ego_pose[1], 0, 0, self.ego_pose[2], 0]

                self.cmd_vel_pub.publish(self.twist_cmd)
                self.controller.args['p'] = ca.vertcat(self.controller.args['x0'])

            
                for i in range(self.controller.N):
                    x_ref = 200
                    y_ref = 3.5
                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], x_ref, y_ref, 0,  0, 0, 0)
                    # for _ in range(1, 2 + len(self.vehicle.vehicle)):
                    #     self.controller.args['p'] = ca.vertcat(self.controller.args['p'], 0)

                # optimization variable current state
                self.controller.args['x0'] = ca.vertcat(
                    ca.reshape(X0, self.controller.nx*(self.controller.N+1), 1),
                    ca.reshape(self.u0, self.controller.nu*self.controller.N, 1)
                )
                if self.obstacle_extraction.obstacle_ahead or self.overtaken is False:
                    leftmost_boundary = np.array(self.obstacle_extraction.leftmost_boundary).reshape(2,)
                    predicted_velocity = np.array(self.obstacle_extraction.predicted_velocity).reshape(2,)
                    rospy.loginfo("leftmost_boundary: {}".format(leftmost_boundary))

                    threshold = 0.5

                    if (leftmost_boundary[0] + threshold <= self.ego_pose[0]):
                        rospy.loginfo("OVERTAKE DETECTED")
                        # rospy.loginfo("OVERTAKE DETECTED ego position: {} obstacle position: {}".format(self.ego_pose, leftmost_boundary))
                        self.overtaken = True

                        leftmost_boundary = leftmost_boundary_stack
                        leftmost_boundary[0] = leftmost_boundary[0] + 0.8
                        leftmost_boundary[1] = leftmost_boundary[1] - 0.2
                        predicted_velocity = predicted_velocity_stack
                    else:
                        leftmost_boundary_stack = leftmost_boundary
                        predicted_velocity_stack = predicted_velocity
                        self.overtaken = False
                    
                    # rospy.loginfo("predicted_velocity: {}".format(predicted_velocity))
                    
                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], leftmost_boundary[0])
                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], leftmost_boundary[1])
                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], predicted_velocity[0])
                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], predicted_velocity[1])
                    
                    sol = self.controller.solver_constrained(
                        x0 = self.controller.args['x0'],
                            lbx = self.controller.args['lbx'],
                            ubx = self.controller.args['ubx'],
                            lbg = self.controller.args['lbg'],
                            ubg = self.controller.args['ubg'],
                            p = self.controller.args['p']
                        )
                    # rospy.loginfo("SOLVER CONSTRAINT: {}".format(self.controller.solver_constrained.stats()['return_status']))
                    self.obstacle_extraction.obstacle_ahead = False

                else:
                    # rospy.loginfo("NO OBSTACLE AHEAD")
                    sol = self.controller.solver_unconstrained(
                            x0 = self.controller.args['x0'],
                            lbx = self.controller.args['lbx'],
                            ubx = self.controller.args['ubx'],
                            lbg = self.controller.args['lbg'],
                            ubg = self.controller.args['ubg'],
                            p = self.controller.args['p']
                        )
                    # rospy.loginfo("SOLVER UNCONSTRAINT: {}".format(self.controller.solver_unconstrained.stats()['return_status']))
                
                # print(self.controller.solver_unconstrained.stats()['return_status'])
                u = ca.reshape(sol['x'][self.controller.nx * (self.controller.N+1):], self.controller.nu, self.controller.N)
                X0 = ca.reshape(sol['x'][: self.controller.nx * (self.controller.N+1)], self.controller.nx, self.controller.N+1)
                # print(X0)
                # print(X0.shape)

                self.u0 = u
                self.twist_cmd.linear.x = X0[2, 1]
                self.twist_cmd.angular.z = X0[3, 1]

                self.x0 = X0[:,1]
                self.x0[0] = self.ego_pose[0]
                self.x0[1] = self.ego_pose[1]
                self.x0[2] = self.ego_pose[2]
                self.cmd_vel_pub.publish(self.twist_cmd)
                command = Float64()
                command.data = X0[4, 1] - X0[5, 1]
                # command.data = self.controller.args['x0'][4] - self.controller.args['x0'][5]
                self.hitch_angle_pub.publish(command)

                if self.save_data:
                    # Store states, inputs, and time in arrays
                    self.state_array.append(self.x0.full())
                    self.input_array.append(u.full())
                    self.time_array.append(rospy.Time.now().to_sec())  # Use current time

                self.rate.sleep()

        finally:
            if self.save_data:
                print("SAVING EGO VEHICLE DATA ...")
                # Save the collected data to numpy arrays
                state_array = np.array(self.state_array)
                input_array = np.array(self.input_array)
                time_array = np.array(self.time_array)

                np.save('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_state_array.npy', state_array)
                np.save('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_input_array.npy', input_array)
                np.save('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/ego_time_array.npy', time_array)

class MoveAndPrintPosition:
    def __init__(self):
        rospy.init_node('move_and_print_position', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 3.0   # Set the desired linear velocity
        self.twist_cmd.angular.z = 0.0  # No angular velocity

        self.ego_pose = (0.0, 0.0)  # Initialize the position

    def odom_callback(self, msg):
        self.ego_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def move_straight(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist_cmd)
            rospy.loginfo("Current position: x = {:.2f}, y = {:.2f}".format(*self.ego_pose))
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