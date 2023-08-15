#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from numpy.linalg import inv, norm
import numpy.linalg as la
from numpy import linalg
from ekf import ExtendedKalmanFilter

from utils import *


class ObstacleExtraction(object):
    def __init__(self):
        rospy.init_node('obstacle_extraction', anonymous=True)
        self.scan_sub = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/ego_vehicle/odom', Odometry, self.odometry_callback)
        self.rate = rospy.Rate(10)

        self.ego_pose = (0.0, 0.0, 0.0)  # Initialize the position
        self.leftmost_boundary = [-float('inf'), 0]

        self.ekf = ExtendedKalmanFilter(
            initial_state=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Initial state: [position_x, position_y, velocity_x, velocity_y, acceleration_x, acceleration_y]
            initial_covariance=np.eye(6) * 1.0,  # Initial covariance matrix
            process_noise_covariance=np.eye(6) * 0.01,  # Process noise covariance
            measurement_noise_covariance=np.eye(2) * 0.1  # Measurement noise covariance
        )

    
    def scan_callback(self, msg):
        threshold = 40  # Adjust the desired threshold
        angle_range = 100  # Range of angles to consider for ellipse computation
        middle_index = len(msg.ranges) // 2

        if any(distance < threshold for distance in msg.ranges[middle_index - angle_range//2:middle_index + angle_range//2]):
            # Filter lidar points within the desired range and remove points with distance infinity
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            lidar_ranges = msg.ranges[middle_index - angle_range//2:middle_index + angle_range//2]
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
                        # point[0] * np.cos(self.ego_pose[2]) - point[1] * np.sin(self.ego_pose[2]) + self.ego_pose[0],
                        # point[0] * np.sin(self.ego_pose[2]) + point[1] * np.cos(self.ego_pose[2]) + self.ego_pose[1]
                        point[0] + self.ego_pose[0],
                        point[1] + self.ego_pose[1]
                    ]
                    for point in lidar_points
                ]

                center, radii, rotation = mvee(np.array(transformed_points))
                alpha = np.arccos(rotation[0, 0]) # from rotation matrix to angle rotation
                leftmost_boundary = retrieve_leftmost_boundary(center, radii, alpha)

                # compute displacement
                displacement = np.linalg.norm(np.array(leftmost_boundary) - np.array(self.leftmost_boundary))
                if displacement > 0.2:
                    print('Dynamic Obstacle Detected')

                    # EKF prediction
                    delta_t = 0.1  # Adjust the time step as needed
                    
                    # Estimate velocity from position differences
                    current_position = leftmost_boundary
                    measured_position = current_position
                    prev_position = self.leftmost_boundary
                    measured_velocity = (np.array(current_position) - np.array(prev_position)) / delta_t

                    # Update EKF
                    self.ekf.predict(delta_t)
                    self.ekf.update(measured_position)
                    
                    predicted_state = self.ekf.get_state()
                    predicted_velocity = predicted_state[2:4]  # Extract velocity component
                    self.prev_velocity = measured_velocity
                    print('Predicted Obstacle Velocity (x, y):', predicted_velocity)
            
                else:
                    print('Static Obstacle Detected')
                self.leftmost_boundary = leftmost_boundary
                print('Leftmost Boundary Point:', self.leftmost_boundary)
            
            rospy.sleep(0.1)

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.ego_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])





if __name__ == '__main__':
    try:
        obstacle_extraction = ObstacleExtraction()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass