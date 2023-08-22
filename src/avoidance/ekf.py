from scipy.linalg import block_diag
from numpy.linalg import inv
import numpy as np

class ExtendedKalmanFilter(object):
    def __init__(self, initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance):
        self.state = initial_state.reshape((6, 1))
        self.covariance = initial_covariance
        self.process_noise_covariance = process_noise_covariance
        self.measurement_noise_covariance = measurement_noise_covariance

    def predict(self, delta_t):
        A = np.array([[1, 0, delta_t, 0, 0.5 * delta_t ** 2, 0],
                      [0, 1, 0, delta_t, 0, 0.5 * delta_t ** 2],
                      [0, 0, 1, 0, delta_t, 0],
                      [0, 0, 0, 1, 0, delta_t],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])  # State transition matrix

        B = np.array([[0.5 * delta_t ** 2], [0.5 * delta_t ** 2], [delta_t], [delta_t], [1], [1]])  # Control input matrix
        # Q = block_diag(self.process_noise_covariance, np.zeros((4, 4)))  # Process noise covariance
        Q = self.process_noise_covariance

        # Prediction step
        self.state = A @ self.state  # x = Ax + Bu
        self.covariance = A @ self.covariance @ A.T + Q  # P = APA' + Q

    def update(self, measurement):
        measurement = np.array(measurement).reshape((2, 1))
        H = np.array([[1, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0]])  # Measurement matrix
        R = self.measurement_noise_covariance  # Measurement noise covariance

        # Calculate Kalman gain
        K = self.covariance @ H.T @ inv(H @ self.covariance @ H.T + R)

        # Update step
        error = measurement - H @ self.state  # y = z - Hx
        
        self.state = self.state + K @ error  # x = x + Ky
        self.covariance = (np.eye(6) - K @ H) @ self.covariance  # P = (I - KH)P

    def get_state(self):
        return self.state



# class ExtendedKalmanFilter(object):
#     def __init__(self, initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance):
#         self.state = initial_state
#         self.covariance = initial_covariance
#         self.process_noise_covariance = process_noise_covariance
#         self.measurement_noise_covariance = measurement_noise_covariance

#     def predict(self, delta_t):
#         A = np.array([[1, 0, delta_t, 0, 0.5*delta_t**2, 0],
#                       [0, 1, 0, delta_t, 0, 0.5*delta_t**2],
#                       [0, 0, 1, 0, delta_t, 0],
#                       [0, 0, 0, 1, 0, delta_t],
#                       [0, 0, 0, 0, 1, 0],
#                       [0, 0, 0, 0, 0, 1]])  # State transition matrix

#         B = np.array([[0.5 * delta_t ** 2], [0.5 * delta_t ** 2], [delta_t], [delta_t], [1], [1]])  # Control input matrix
#         # Q = block_diag(self.process_noise_covariance, np.zeros((4, 4)))  # Process noise covariance

#         # print('A', A)
#         # print('Q', Q)

#         # Prediction step
#         self.state = np.dot(A, self.state)  # x = Ax + Bu
#         self.covariance = np.dot(A, np.dot(self.covariance, A.T)) #+ Q  # P = APA' + Q



#     def update(self, measurement):
#         H = np.array([[1, 0, 0, 0, 0, 0],
#                       [0, 1, 0, 0, 0, 0]])  # Measurement matrix
#         R = self.measurement_noise_covariance  # Measurement noise covariance

#         # Calculate Kalman gain
#         K = np.dot(self.covariance, np.dot(H.T, inv(np.dot(H, np.dot(self.covariance, H.T)) + R)))

#         # Update step
#         print('statettttteeeeee', self.state.shape)
#         print('measurement', measurement.shape)
#         print('K', K.shape)
#         print('H', H.shape)

#         innovation = measurement - H @ self.state  # y = z - Hx
#         print('innovation', innovation)
#         self.state = self.state +  # x = x + Ky
#         self.covariance = np.dot(np.eye(6) - np.dot(K, H), self.covariance)  # P = (I - KH)P

#     def get_state(self):
#         return self.state

