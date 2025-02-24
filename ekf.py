import numpy as np
from scipy.spatial.transform import Rotation as R


class EKF:
    def __init__(self, mag_noise, acc_noise):
        self.x = np.array([1, 0., 0., 0., 0, 0, 0, 0, 0, 0])
        self.F = np.zeros((10, 10))
        self.P = np.identity(10)
        self.Q = np.zeros((10, 10))
        Fu = np.matrix([[0, 0, 0, 0, 0, 0], 
              [0, 0, 0, 0, 0, 0], 
              [0, 0, 0, 0, 0, 0], 
              [0, 0, 0, 0, 0, 0], 
              [1, 0, 0, 0, 0, 0], 
              [0, 1, 0, 0, 0, 0], 
              [0, 0, 1, 0, 0, 0], 
              [0, 0, 0, 1, 0, 0], 
              [0, 0, 0, 0, 1, 0], 
              [0, 0, 0, 0, 0, 1]])
        U = np.matrix([[1e-9, 0, 0, 0, 0, 0],
                      [0, 1e-9, 0, 0, 0, 0],
                      [0, 0, 1e-9, 0, 0, 0],
                      [0, 0, 0, 1e-11, 0, 0],
                      [0, 0, 0, 0, 1e-11, 0],
                      [0, 0, 0, 0, 0, 1e-11]])
        self.Q = Fu @ U @ np.matrix.transpose(Fu)
        self.R = [[mag_noise, 0, 0, 0],
             [0, mag_noise, 0, 0],
             [0, 0, mag_noise, 0],
             [0 ,0 , 0, acc_noise]] 
        self.H = np.zeros((4, 10))
        self.K = np.zeros((10, 10))
        self.R = np.zeros(())

        self.h = np.zeros(4)
        self.z = np.zeros(4)
        self.dt = 0.1


    def propogate(self, gyro=np.array, acc=np.array, mag=np.array, dt = float):
        self.dt = dt
        self.x = np.array(self.x).flatten()
        bias = self.x[7:10]
        wx = gyro[0] - bias[0]
        wy = gyro[1] - bias[1]
        wz = gyro[2] - bias[2]
        q = self.x[0:4]
        q_w = q[0]
        q_i = q[1]
        q_j = q[2]
        q_k = q[3]
        R_bn = np.matrix([[q_w ** 2+q_i ** 2-q_j ** 2-q_k ** 2, 2*(q_i*q_j-q_w*q_k), 2*(q_w*q_j+q_i*q_k)],
                [2*(q_i*q_j+q_w*q_k), (q_w ** 2-q_i ** 2+q_j ** 2-q_k ** 2), 2*(q_j*q_k-q_w*q_i)],
                [2*(q_i*q_k-q_w*q_j), 2*(q_w*q_i+q_j*q_k), (q_w ** 2-q_i ** 2-q_j ** 2+q_k ** 2)]])


        # magnetometer
        vct = np.array(R_bn @ mag).flatten()
        mB = np.linalg.inv(R_bn) @ (np.array([vct[0], vct[1], 0]))
        mB = np.array(mB).flatten()
        phy = np.arctan2(- mB[1], mB[0])

        self.z =  np.array([ acc[0],
                        acc[1],
                        acc[2],
                        phy])

        # F
        self.F = [[             1, -0.5*dt*wx, -0.5*dt*wy, -0.5*dt*wz, -0.5*dt*q_i, -0.5*dt*q_j, -0.5*dt*q_k, 0, 0, 0], 
              [  0.5*dt*wx,             1,  0.5*dt*wz, -0.5*dt*wy,  0.5*dt*q_w, -0.5*dt*q_k,  0.5*dt*q_j, 0, 0, 0],
              [  0.5*dt*wy, -0.5*dt*wz,             1,  0.5*dt*wx,  0.5*dt*q_k,  0.5*dt*q_w, -0.5*dt*q_i, 0, 0, 0],
              [  0.5*dt*wz,  0.5*dt*wy, -0.5*dt*wx,             1, -0.5*dt*q_j,  0.5*dt*q_i,  0.5*dt*q_w, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, -1, 0, 0], 
              [0, 0, 0, 0, 0, 0, 0, 0, -1, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, -1],
              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ],
              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0 ],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ]]
        
    def predict(self):
        # predict measurements 
        # Accelerometer
        q = self.x[0:4]
        q_w = q[0]
        q_i = q[1]
        q_j = q[2]
        q_k = q[3]
        self.x = np.array(self.x).flatten()
        bias = self.x[7:10]
        wx = gyro[0] - bias[0]
        wy = gyro[1] - bias[1]
        wz = gyro[2] - bias[2]
        yaw = np.arctan2((2 * (q[1] * q[3] + q[0] * q[2])),(1. - 2 * q[2] ** 2 - 2 * q[3] ** 2))
        self.h = np.array([- 2 * (q[1] * q[3] + q[0] * q[2]),
                      - 2 * (q[2] * q[3] - q[0] * q[1]),
                      - (q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2),
                          yaw ])
        print(self.h)

        # Observation Matrix
        dhm_dqw = (2 * q_i * (-2 * q_j**2 - 2 * q_k**2 + 1)) / (4 * (q_w * q_k + q_i * q_j)**2 + (-2 * q_j**2 - 2 * q_k**2 + 1)**2)
        dhm_dqi = (2 * q_j * (-2 * q_j**2 - 2 * q_k**2 + 1)) / (4 * (q_w * q_k + q_i * q_j)**2 + (-2 * q_j**2 - 2 * q_k**2 + 1)**2)
        dhm_dqj = (2 * (q_w * q_k + q_i * q_j) - 2 * q_i * q_k**2 + q_i) / (4 * (q_w**2 - 1) * q_k**2 + 8 * q_w * q_i * q_j * q_k + 4 * q_j**2 * (q_i**2 + q_k**2 - 1) + 4 * q_i**4 + 4 * q_k**4 + 1)
        dhm_dqk = (q_w * (-4 * q_j**2 + 4 * q_k**2 + 2) + 8 * q_i * q_j * q_k) / (4 * (q_w**2 - 1) * q_k**2 + 8 * q_w * q_i * q_j * q_k + 4 * q_j**2 * (q_i**2 + q_k**2 - 1) + 4 * q_i**4 + 4 * q_k**4 + 1)

        self.H = np.matrix([
            [-2*q_j, -2*q_k, -2*q_w, -2*q_i, 0, 0, 0, 0, 0, 0],
            [ 2*q_i,  2*q_w, -2*q_k, -2*q_j, 0, 0, 0, 0, 0, 0],
            [-2*q_w,  2*q_i,  2*q_j, -2*q_k, 0, 0, 0, 0, 0, 0],
            [dhm_dqw, dhm_dqi, dhm_dqj, dhm_dqk, 0, 0, 0, 0, 0, 0]
        ])
    


        # EKF steps
        trans = np.matrix([[0, - wx, -wy, -wz],
                 [wx, 0, -wz, wy],
                 [wy, wz, 0, -wx],
                 [-wz, -wy, wx, 0]])

        q_kp = q + (0.5 * self.dt) * trans @ q

        self.x[0:4] = q_kp
        self.x[4] = wx
        self.x[5] = wy
        self.x[6] = wz
        self.x[7:10] = bias
        self.x[0:4] = self.x[0:4] / np.linalg.norm(self.x[0:4])
        self.P = self.F @ self.P @ np.matrix.transpose(np.matrix(self.F)) + self.Q

    def update(self):
        v = self.h - self.z
        self.K = self.P @ np.matrix.transpose(np.matrix(self.H)) @ np.linalg.inv(self.H @ self.P @ np.matrix.transpose(np.matrix(self.H)) + self.R)
        self.x = self.x + self.K @ v
        self.x[0:4] = np.array(self.x[0:4] / np.linalg.norm(self.x[0:4]))
        self.P = (np.identity(10) - self.K @ self.H) @ self.P

ekf = EKF(0.00001, 0.000001)
gyro = np.array([0, 0, 0])
acc = np.array([0.001, 0, -1])
mag = np.array([0, 0, 0])
for i in range(100):
    ekf.propogate(gyro, acc, mag, 0.1)
    ekf.predict()
    ekf.update()

