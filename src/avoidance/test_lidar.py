#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from scipy.linalg import fractional_matrix_power
from numpy.linalg import inv, norm
import numpy.linalg as la


def mvee(points, tol=0.0001):
    """
    Finds the ellipse equation in "center form"
    (x-c).T * A * (x-c) = 1
    """
    N, d = points.shape
    Q = np.column_stack((points, np.ones(N))).T
    err = tol+1.0
    u = np.ones(N)/N
    while err > tol:
        # assert u.sum() == 1 # invariant
        X = np.dot(np.dot(Q, np.diag(u)), Q.T)
        M = np.diag(np.dot(np.dot(Q.T, la.inv(X)), Q))
        jdx = np.argmax(M)
        step_size = (M[jdx]-d-1.0)/((d+1)*(M[jdx]-1.0))
        new_u = (1-step_size)*u
        new_u[jdx] += step_size
        err = la.norm(new_u-u)
        u = new_u
    c = np.dot(u, points)
    A = la.inv(np.dot(np.dot(points.T, np.diag(u)), points)
               - np.multiply.outer(c, c))/d
    return A, c

def odom_callback(odom_msg):
    global ego_pose
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
    ego_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])

def callback(scan_msg):
    global ego_pose
    # Check if an obstacle is detected based on your criteria
    threshold = 10  # Adjust the desired threshold
    angle_range = 40  # Range of angles to consider for ellipse computation
    middle_index = len(scan_msg.ranges) // 2
    
    if any(distance < threshold for distance in scan_msg.ranges[middle_index - angle_range//2:middle_index + angle_range//2]):
        # print('Obstacle Ahead Detected')
        
        # Filter lidar points within the desired range and remove points with distance infinity
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        lidar_ranges = scan_msg.ranges[middle_index - angle_range//2:middle_index + angle_range//2]
        lidar_points = [
            [
                lidar_ranges[i] * np.cos(angle_min + angle_increment * (middle_index - angle_range//2 + i)),
                lidar_ranges[i] * np.sin(angle_min + angle_increment * (middle_index - angle_range//2 + i))
            ]
            for i in range(angle_range)
            if lidar_ranges[i] != float('inf')  # Filter out points with distance infinity
        ]
        
        if len(lidar_points) > 0:
            # Transform lidar points to ego vehicle frame using odometry
            transformed_points = [
                [
                    point[0] * np.cos(ego_pose[2]) - point[1] * np.sin(ego_pose[2]) + ego_pose[0],
                    point[0] * np.sin(ego_pose[2]) + point[1] * np.cos(ego_pose[2]) + ego_pose[1]
                ]
                for point in lidar_points
            ]

            # print('Lidar Points:', transformed_points)
            
            # # Compute the Minimum-Volume Enclosing Ellipsoid (MVEE)
            # center, lengths = compute_mvee(transformed_points)
            # print('Ellipsoid Center:', center)
            # print('Ellipsoid Lengths:', lengths)

            # center, A = khachiyan_algorithm(transformed_points)
            A, center = mvee(np.array(transformed_points))

            print("Center:", center)
            print("A matrix:", A)

            rospy.sleep(10)

rospy.init_node('ellipse_drawer')

ego_pose = (0, 0, 0)  # Initialize ego pose

sub_odom = rospy.Subscriber('/ego_vehicle/odom', Odometry, odom_callback)
sub_lidar = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, callback)
rospy.spin()
