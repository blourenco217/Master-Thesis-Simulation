import numpy as np
from numpy import linalg
import rospy

def quadratic_equation(a, b, c):
    if b**2 - 4*a*c < 0:
        return []
    else:
        return [(-b + np.sqrt(b**2 - 4*a*c))/(2*a), (-b - np.sqrt(b**2 - 4*a*c))/(2*a)]
    
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

    if center[1] >= 0:
        theta_min = 0
        theta_max = np.pi / 2
    else:
        theta_min = -np.pi / 2
        theta_max = 0
    leftmost_boundary = [-float('inf'), 0]

    while theta_max - theta_min > 1e-6:
        theta_m = (theta_min + theta_max) / 2

        intersection_points = line_ellipse_intersection(center, radii, rotation, theta_m)
        # rospy.loginfo('THETA: {}'.format(theta_m))
        # rospy.loginfo('INTERSECTION POINTS: {}\n'.format(intersection_points))

        if len(intersection_points) == 1:
            leftmost_boundary = intersection_points
            break
        elif (len(intersection_points) == 2 and theta_m >= 0) or (len(intersection_points) == 0 and theta_m < 0):
            theta_min = theta_m
            if len(intersection_points) == 2:
                leftmost_boundary = [intersection_points[0]]
        else:
            theta_max = theta_m
            if len(intersection_points) == 2:
                leftmost_boundary = [intersection_points[0]]

    return leftmost_boundary