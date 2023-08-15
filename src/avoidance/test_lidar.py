#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from numpy.linalg import inv, norm
import numpy.linalg as la
from numpy import linalg

from sklearn.cluster import KMeans

def mvee(P, tolerance=0.01):
    """ Find the minimum volume ellipsoid which holds all the points
    
    Based on work by Nima Moshtagh
    http://www.mathworks.com/matlabcentral/fileexchange/9542
    and also by looking at:
    http://cctbx.sourceforge.net/current/python/scitbx.math.minimum_covering_ellipsoid.html
    Which is based on the first reference anyway!
    
    Here, P is a numpy array of N dimensional points like this:
    P = [[x,y,z,...], <-- one point per line
            [x,y,z,...],
            [x,y,z,...]]
    
    Returns:
    (center, radii, rotation)
    
    """
    (N, d) = np.shape(P)
    d = float(d)

    # Q will be our working array
    Q = np.vstack([np.copy(P.T), np.ones(N)]) 
    QT = Q.T
    
    # initializations
    err = 1.0 + tolerance
    u = (1.0 / N) * np.ones(N)

    # Khachiyan Algorithm
    while err > tolerance:
        V = np.dot(Q, np.dot(np.diag(u), QT))
        M = np.diag(np.dot(QT , np.dot(linalg.inv(V), Q)))    # M the diagonal vector of an NxN matrix
        j = np.argmax(M)
        maximum = M[j]
        step_size = (maximum - d - 1.0) / ((d + 1.0) * (maximum - 1.0))
        new_u = (1.0 - step_size) * u
        new_u[j] += step_size
        err = np.linalg.norm(new_u - u)
        u = new_u

    # center of the ellipse 
    center = np.dot(P.T, u)

    # the A matrix for the ellipse
    A = linalg.inv(
                    np.dot(P.T, np.dot(np.diag(u), P)) - 
                    np.array([[a * b for b in center] for a in center])
                    ) / d
    
    U, s, rotation = linalg.svd(A)
    radii = 1.0/np.sqrt(s)
    return (center, radii, rotation)


def quadratic_equation(a, b, c):
    # print('a, b, c', a, b, c)
    # print('b**2 - 4*a*c', b**2 - 4*a*c)
    if b**2 - 4*a*c < 0:
        return []
    else:
        return [(-b + np.sqrt(b**2 - 4*a*c))/(2*a), (-b - np.sqrt(b**2 - 4*a*c))/(2*a)]

def line_ellipse_intersection(center, radii, alpha, beta):

    x_0, y_0 = center[0], center[1]
    a, b = radii[0], radii[1]

    A = (np.cos(alpha)**2)/a**2 + (np.sin(alpha)**2)/b**2
    B = 2*np.cos(alpha)*np.sin(alpha)*(1/a**2 - 1/b**2)
    C = (np.sin(alpha)**2)/a**2 + (np.cos(alpha)**2)/b**2


    eq_a = A + B*np.tan(beta) + C*np.tan(beta)**2
    eq_b = - 2*A*x_0 - B*(x_0*np.tan(beta) + y_0) - 2*C*y_0*np.tan(beta)
    eq_c = A*x_0**2 + B*x_0*y_0 + C*y_0**2 - 1
    solution_x = quadratic_equation(eq_a, eq_b, eq_c)
    solution_y = [np.tan(beta)*x + y_0 for x in solution_x]
    solutions = [[x, y] for x, y in zip(solution_x, solution_y)]
    # print('solutions', solutions)

    if len(solutions) == 0:
        return []
    elif np.linalg.norm(np.array(solutions[0]) - np.array(solutions[1])) < 1e-3:
        return [solutions[0]]
    else:
        return solutions


def retrieve_leftmost_boundary(center, radii, rotation):
    theta_min = 0
    theta_max = np.pi / 2
    leftmost_boundary = [-float('inf'), 0]

    while theta_max - theta_min > 1e-6:
        theta_m = (theta_min + theta_max) / 2

        intersection_points = line_ellipse_intersection(center, radii, rotation, theta_m)

        if len(intersection_points) == 1:
            leftmost_boundary = intersection_points
            break
        elif len(intersection_points) == 2:
            theta_min = theta_m
        else:
            theta_max = theta_m

    return leftmost_boundary


def local_planner(ego_pose, lidar_points_prev, scan_msg):
    # Check if an obstacle is detected based on your criteria
    threshold = 25  # Adjust the desired threshold
    angle_range = 100  # Range of angles to consider for ellipse computation
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

            print('Lidar Points:', transformed_points)
            # Classify obstacles
            if lidar_points_prev != []:

                print('Lidar Points Prev:', lidar_points_prev)
                static_obstacles, dynamic_obstacles = classify_obstacles(transformed_points)
                print('Static Obstacles:', static_obstacles)
                print('Dynamic Obstacles:', dynamic_obstacles)
                
            
            # Compute the Minimum-Volume Enclosing Ellipsoid (MVEE)
            center, radii, rotation = mvee(np.array(transformed_points))

            alpha = np.arccos(rotation[0, 0])
            # print('Center:', center)
            # print('Radii:', radii)
            # print('Alpha:', alpha)


            # print("Center:", center)
            # print("A matrix:", A)

            leftmost_boundary = retrieve_leftmost_boundary(center, radii, alpha)
            print('Leftmost Boundary Point:', leftmost_boundary)

        lidar_points_prev = transformed_points.copy()
        
        rospy.sleep(5)

def classify_obstacles(points):
    # Separate points into static and dynamic obstacles using k-means clustering
    num_clusters = 2  # Number of clusters for classification
    kmeans = KMeans(n_clusters=num_clusters, n_init=10)
    labels = kmeans.fit_predict(points)

    static_obstacles = []
    dynamic_obstacles = []

    static_threshold = 0.1   # Threshold for static obstacles - empirical value
    dynamic_threshold = 0.5  # Threshold for dynamic obstacles - empirical value

    for i in range(num_clusters):
        cluster_points = [point for j, point in enumerate(points) if labels[j] == i]
        
        # Calculate displacement for cluster points
        # print('Cluster Points:', cluster_points)
        # print('Lidar Points Prev:', lidar_points_prev)
        # quit()
        displacement = np.linalg.norm(np.array(cluster_points) - np.array(lidar_points_prev))
        
        if np.max(displacement) < static_threshold:
            static_obstacles.extend(cluster_points)
        elif np.min(displacement) > dynamic_threshold:
            dynamic_obstacles.extend(cluster_points)

    return static_obstacles, dynamic_obstacles


def odom_callback(odom_msg):
    global ego_pose
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
    ego_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])


def callback(scan_msg):
    global ego_pose
    global lidar_points_prev
    lidar_points_prev = []
    local_planner(ego_pose, lidar_points_prev, scan_msg)
    # print('ççççççççç')

    # min_points_in_cluster = 3  # Minimum number of points in a cluster to consider as an obstacle
    # max_distance_from_ego = 20  # Maximum distance from ego vehicle to classify an obstacle as static
    # # Classify obstacles using k-means clustering
    # num_clusters = 2  # Adjust as needed
    # kmeans = KMeans(n_clusters=num_clusters)
    # labels = kmeans.fit_predict(transformed_points)

    # # Extract cluster centers
    # cluster_centers = kmeans.cluster_centers_

    # # Classify obstacles as static or dynamic
    # static_obstacles = []
    # dynamic_obstacles = []
    # for i in range(num_clusters):
    #     if len(labels[labels == i]) >= min_points_in_cluster:  # Adjust min_points_in_cluster as needed
    #         if np.linalg.norm(cluster_centers[i] - ego_pose[:2]) < max_distance_from_ego:  # Adjust max_distance_from_ego as needed
    #             static_obstacles.append(cluster_centers[i])
    #         else:
    #             dynamic_obstacles.append(cluster_centers[i])
    
    # print('Static Obstacles:', static_obstacles)
    # print('Dynamic Obstacles:', dynamic_obstacles)

            

rospy.init_node('ellipse_drawer')

ego_pose = (0, 0, 0)  # Initialize ego pose

sub_odom = rospy.Subscriber('/ego_vehicle/odom', Odometry, odom_callback)
sub_lidar = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, callback)
rospy.spin()
