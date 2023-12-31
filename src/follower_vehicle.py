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
from control_msgs.msg import JointControllerState
import sys 
import time

class FollowerCACCNode(object):
    def __init__(self, namespace = 'follower_vehicle'):
        rospy.init_node('follower_cacc_node', anonymous=True)

        self.follower_id = rospy.get_param('/' + namespace + '/follower_id')
        rospy.loginfo("Follower id: %d", self.follower_id)

        if self.follower_id > 1:
            # subscriber to proceder vehicle
            namespace_proceder = 'follower_vehicle_' + str(self.follower_id - 1)
            rospy.Subscriber('/' + namespace_proceder + '/odom', Odometry, self.odom_proceder_callback)
            rospy.Subscriber('/' + namespace_proceder + '/cmd_vel', Twist, self.cmd_vel_proceder_callback)
            self.proceder_pose = (0.0, 0.0, 0.0)
            self.proceder_vel = [0.0,0.0,0.0]
            rospy.loginfo("ATTENZIONE PICKPOCKET")

        
        # Parameters
        self.target_headway = 1.0  # Desired time gap between vehicles (in seconds)
        self.kp = 0.5              # Proportional gain
        
        # Subscribers
        rospy.Subscriber('/ego_vehicle/odom', Odometry, self.odom_ego_callback)
        rospy.Subscriber('/ego_vehicle/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/' + namespace + '/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/' + namespace + '/odom', Odometry, self.odometry_follower_callback)
        
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
        self.hitch_angle_sub = rospy.Subscriber('/follower_vehicle/hitch_joint_position_controller/state', JointControllerState, self.hitch_angle_callback)
        self.hitch_angle = 0

        self.save_data = True
        self.state_array = []
        self.input_array = []
        self.time_array = []
    
    def cmd_vel_proceder_callback(self, cmd_vel_msg):
        self.proceder_twist = cmd_vel_msg
    
    def odom_proceder_callback(self, odom_msg):
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        self.proceder_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])
        self.proceder_vel = (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z)

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
        try:
            while not rospy.is_shutdown():
                X0 = ca.repmat(self.x0, 1, self.controller.N+1)
                self.controller.args['x0'] = [self.follower_pose[0], self.follower_pose[1], float(self.x0[2]), float(self.x0[3]), self.follower_pose[2], float(self.x0[5])]
                self.cmd_vel_pub.publish(self.follower_twist)
                self.controller.args['p'] = ca.vertcat(self.controller.args['x0'])

                for i in range(self.controller.N):
                    # self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.ego_pose[0], self.ego_pose[1], 
                    #                                        np.sqrt(self.ego_vel[0]**2 + self.ego_vel[1]**2), self.ego_vel[2],
                    #                                          self.ego_pose[2], 0)

                    vel_ref = np.sqrt(self.ego_vel[0]**2 + self.ego_vel[1]**2)

                    length_truck = 10

                    # if vel_ref < 2:
                    #     # rospy.loginfo("ATTENZIONNEEE PICKPOCKET")
                    #     vel_ref = 0
                    #     x_ref = self.follower_pose[0] - length_truck * np.cos(self.follower_pose[2])
                    #     y_ref = self.follower_pose[1] - length_truck * np.sin(self.follower_pose[2])
                    # else:

                    #     if self.follower_id > 1:
                    #         x_ref = self.proceder_pose[0] + self.proceder_vel[0] * self.controller.dt * (i+1) - length_truck * np.cos(self.proceder_pose[2])
                    #         y_ref = self.proceder_pose[1] + self.proceder_vel[1] * self.controller.dt * (i+1) - length_truck * np.sin(self.proceder_pose[2])
                    #     else:
                    #         x_ref = self.ego_pose[0] + self.ego_vel[0] * self.controller.dt * (i+1) - length_truck * np.cos(self.ego_pose[2])
                    #         y_ref = self.ego_pose[1] + self.ego_vel[1] * self.controller.dt * (i+1) - length_truck * np.sin(self.ego_pose[2])
                    
                    if self.follower_id > 1:
                            x_ref = self.proceder_pose[0] + self.proceder_vel[0] * self.controller.dt * (i+1) - length_truck * np.cos(self.proceder_pose[2])
                            y_ref = self.proceder_pose[1] + self.proceder_vel[1] * self.controller.dt * (i+1) - length_truck * np.sin(self.proceder_pose[2])
                    else:
                        x_ref = self.ego_pose[0] + self.ego_vel[0] * self.controller.dt * (i+1) - length_truck * np.cos(self.ego_pose[2])
                        y_ref = self.ego_pose[1] + self.ego_vel[1] * self.controller.dt * (i+1) - length_truck * np.sin(self.ego_pose[2])

                        

                    self.controller.args['p'] = ca.vertcat(self.controller.args['p'], x_ref, y_ref, vel_ref, 0, 0, 0)
                
                # for i in range(self.controller.N):
                #     if self.follower_id > 1:
                #         self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.proceder_pose[0], self.proceder_pose[1])
                #     else:
                #         self.controller.args['p'] = ca.vertcat(self.controller.args['p'], self.ego_pose[0], self.ego_pose[1])

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
                

                if vel_ref < 0.6:
                    time_ego_stopped = time.time()
                    u = ca.DM.zeros((self.controller.nu, self.controller.N))
                    time_detected = time.time()
                    rospy.loginfo("FOLLOWER {} DETECTED EGO STOPPED IN {}".format(self.follower_id, time_detected - time_ego_stopped))
                else:
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
                self.cmd_vel_pub.publish(self.follower_twist)
                command = Float64()
                command.data = X0[4, 1] - X0[5, 1]
                self.hitch_angle_pub.publish(command)


                if self.save_data:
                    # Store states, inputs, and time in arrays
                    self.state_array.append(self.x0.full())
                    self.input_array.append(u.full())
                    self.time_array.append(rospy.Time.now().to_sec())  # Use current time
                
                self.rate.sleep()
        finally:
            if self.save_data:
                print("SAVING FOLLOWER {} VEHICLE DATA ...".format(self.follower_id))
                # Save the collected data to numpy arrays
                state_array = np.array(self.state_array)
                input_array = np.array(self.input_array)
                time_array = np.array(self.time_array)

                np.save('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_'+ str(self.follower_id) +'_state_array.npy', state_array)
                np.save('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_'+ str(self.follower_id) +'_input_array.npy', input_array)
                np.save('/media/psf/simulation/catkin_ws/src/my_truckie/results/arrays/follower_'+ str(self.follower_id) +'_time_array.npy', time_array)

if __name__ == '__main__':
    try:
        namespace = sys.argv[1]             # Get namespace from command line argument
        follower_node = FollowerCACCNode(namespace)
        follower_node.control_loop()
    except rospy.ROSInterruptException:
        pass
