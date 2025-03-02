import numpy as np
import matplotlib.pyplot as plt
import scipy.io


def print_norm(mat):
    print("norm: %.30f" % sum(np.array(mat).flatten()))


def euler2quat(yaw, pitch, roll):
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    q = np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr
    ])
    return q


def quat2euler(q):
    q0, q1, q2, q3 = q.T
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    pitch = np.arcsin(2 * (q0 * q2 - q3 * q1))
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
    return yaw, pitch, roll


class EKF:
    def __init__(self, gyro, acc, mag):
        num_init = 100

        mean_gyro = np.mean(gyro[0:num_init], axis=0)
        acc = np.array(acc)
        mean_acc = np.mean(-acc[0:num_init], axis=0)
        mean_mag = np.mean(mag[0:num_init], axis=0)

        roll_init = np.atan2(mean_acc[1], mean_acc[2])
        pitch_init = np.atan2(-mean_acc[0], np.sqrt(mean_acc[1]*mean_acc[1]+mean_acc[2]*mean_acc[2]))

        cp = np.cos(pitch_init)
        sp = np.sin(pitch_init)
        cr = np.cos(roll_init)
        sr = np.sin(roll_init)

        self.R = np.matrix([[cp,  sp*sr, sp*cr],
                            [0,   cr,    -sr],
                            [-sp, cp*sr, cp*cr]])
        mr = np.array(self.R @ mean_mag).flatten()
        yaw_init = np.atan2(-mr[1], mr[0])

        q_init = euler2quat(yaw_init, roll_init, pitch_init)

        self.x = np.array([q_init[0],
                           q_init[1],
                           q_init[2],
                           q_init[3],
                           0,
                           0,
                           0,
                           mean_gyro[0],
                           mean_gyro[1],
                           mean_gyro[2]])
        self.y = np.array([0,0,0,0])

        self.P = np.matrix([[10, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 10, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 10, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 10, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 10, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 10, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 10, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 10, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 10, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 10]])
        v_bias = 1.0e-11

        gyro_sum = np.zeros(3)
        acc_sum = np.zeros(3)
        mag_sum = 0

        for i in range(num_init):
            gyro_sum = gyro_sum + (gyro[i] - mean_gyro) ** 2
            acc_sum = acc_sum + (acc[i] + mean_acc) ** 2

            mw = np.array(self.R @ mag[i]).flatten()
            mag_yaw = np.atan2(-mw[1], mw[0])
            mag_sum = mag_sum + (mag_yaw - yaw_init) * (mag_yaw - yaw_init)

        s_gyro = (np.sqrt(gyro_sum[0]/(num_init-1)) + np.sqrt(gyro_sum[1]/(num_init-1)) + np.sqrt(gyro_sum[2]/(num_init-1))) / 3
        s_acc = (np.sqrt(acc_sum[0]/(num_init-1)) + np.sqrt(acc_sum[1]/(num_init-1)) + np.sqrt(acc_sum[2]/(num_init-1))) / 3
        s_yaw = np.sqrt(mag_sum/(num_init-1))

        self.R = np.matrix([[2.12425429e-06, 0, 0, 0],
                            [0, 2.12425429e-06, 0, 0],
                            [0, 0, 2.12425429e-06, 0],
                            [0, 0, 0, 7.98243297e-05]])
        print(self.R)
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

        q0 = self.x[0]
        q1 = self.x[1]
        q2 = self.x[2]
        q3 = self.x[3]
        self.R_bn = np.matrix([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                               [2*(q1*q2+q0*q3), (q0**2-q1**2+q2**2-q3**2), 2*(q2*q3-q0*q1)],
                               [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), (q0**2-q1**2-q2**2+q3**2)]])

        self.R_nb = np.transpose(self.R_bn)

        U = np.diag([3.47930986e-04, 3.47930986e-04, 3.47930986e-04, v_bias, v_bias, v_bias])
        self.Q = Fu @ U @ np.transpose(Fu)
        print(self.Q)

    def update_mag(self, mag):
        ymag = mag
        mn = np.array(self.R_bn @ ymag).flatten()
        mnh = np.array([mn[0], mn[1], 0])
        mb = np.array(self.R_nb @ mnh).flatten()
        yyaw = np.atan2(-mb[1], mb[0])

        self.y = np.array([self.y[0], self.y[1], self.y[2], yyaw])

    def update_acc(self, acc):
        yacc = acc
        self.y = np.array([yacc[0], yacc[1], yacc[2], self.y[3]])

    def quaternion_multiply(self, q, r):
        q0, q1, q2, q3 = q
        r0, r1, r2, r3 = r
        return np.array([
            q0*r0 - q1*r1 - q2*r2 - q3*r3,
            q0*r1 + q1*r0 + q2*r3 - q3*r2,
            q0*r2 - q1*r3 + q2*r0 + q3*r1,
            q0*r3 + q1*r2 - q2*r1 + q3*r0
        ])

    def quaternion_update(self, q, wx, wy, wz, dt):
        omega = np.array([0, wx, wy, wz])
        q_dot = 0.5 * self.quaternion_multiply(q, omega)
        q_new = q + dt * q_dot

        return q_new / np.linalg.norm(q_new)

    def predict(self, gyro,dt):
        self.x = np.array(self.x).flatten()

        q0 = self.x[0]
        q1 = self.x[1]
        q2 = self.x[2]
        q3 = self.x[3]
        wx = self.x[4]
        wy = self.x[5]
        wz = self.x[6]
        xgx = self.x[7]
        xgy = self.x[8]
        xgz = self.x[9]

        q = self.quaternion_update(self.x[0:4], wx, wy, wz, dt)
        self.x[0] = q[0]
        self.x[1] = q[1]
        self.x[2] = q[2]
        self.x[3] = q[3]
        self.x[0] = q0+0.5*dt*(-q1*wx-q2*wy-q3*wz)
        self.x[1] = q1+0.5*dt*(q0*wx-q3*wy+q2*wz)
        self.x[2] = q2+0.5*dt*(q3*wx+q0*wy-q1*wz)
        self.x[3] = q3+0.5*dt*(-q2*wx+q1*wy+q0*wz)
        self.x[4] = gyro[0] - xgx
        self.x[5] = gyro[1] - xgy
        self.x[6] = gyro[2] - xgz
        self.x[7] = xgx
        self.x[8] = xgy
        self.x[9] = xgz

        self.x[0:4] = self.x[0:4] / np.linalg.norm(self.x[0:4])

        self.F = np.matrix([[1., -0.5*dt*wx, -0.5*dt*wy, -0.5*dt*wz, -0.5*dt*q1, -0.5*dt*q2, -0.5*dt*q3, 0, 0, 0],
                            [0.5*dt*wx,             1.,  0.5*dt*wz, -0.5*dt*wy,  0.5*dt*q0, -0.5*dt*q3,  0.5*dt*q2, 0, 0, 0],
                            [0.5*dt*wy, -0.5*dt*wz,             1.,  0.5*dt*wx,  0.5*dt*q3,  0.5*dt*q0, -0.5*dt*q1, 0, 0, 0],
                            [0.5*dt*wz,  0.5*dt*wy, -0.5*dt*wx,             1., -0.5*dt*q2,  0.5*dt*q1,  0.5*dt*q0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, -1., 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, -1., 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, -1.],
                            [0, 0, 0, 0, 0, 0, 0, 1., 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1., 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1.]])

        self.P = self.F @ self.P @ np.transpose(self.F) + self.Q

        q0 = self.x[0]
        q1 = self.x[1]
        q2 = self.x[2]
        q3 = self.x[3]

        self.R_bn = np.matrix([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                               [2*(q1*q2+q0*q3), (q0**2-q1**2+q2**2-q3**2), 2*(q2*q3-q0*q1)],
                               [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), (q0**2-q1**2-q2**2+q3**2)]])

        self.R_nb = np.transpose(self.R_bn)

        zacc = np.array(self.R_nb @ np.array([0, 0, -1])).flatten()

        zyaw = quat2euler(self.x[0:4])[0]

        self.z = np.array([zacc[0], zacc[1], zacc[2], zyaw])

        dhm_dqw = (2 * q3 *(1 - 2 *(q2**2 + q3**2)))/(4 *(q1 *q2 + q0 *q3)**2 + (1 - 2 *(q2**2 + q3**2))**2);
        dhm_dqi = (2 * q2 *(1 - 2 *(q2**2 + q3**2)))/(4 *(q1 * q2 + q0 * q3)**2 + (1 - 2 * (q2**2 + q3**2))**2);
        dhm_dqj = (2 * (q1 + 2 *q1 *q2**2 + 4 *q0 *q2 *q3 - 2 *q1 *q3**2))/(1 + 4 *q2**4 + 8 * q0 * q1 * q2 * q3 + 4 *(-1 + q0**2)* q3**2 + 4* q3**4 + 4 *q2**2 * (-1 + q1**2 + 2 * q3**2));
        dhm_dqk = (8 * q1 * q2 * q3 + q0 * (2 - 4 * q2**2 + 4 * q3**2))/(1 + 4 * q2**4 + 8 * q0 * q1 * q2 * q3 + 4 * (-1 + q0**2) * q3**2 + 4 * q3**4 + 4 * q2**2 * (-1 + q1**2 + 2 * q3**2));

        self.H = np.matrix([
            [2*q2, -2*q3, 2*q0, -2*q1, 0, 0, 0, 0, 0, 0],
            [- 2*q1,  -2*q0, -2*q3, -2*q2, 0, 0, 0, 0, 0, 0],
            [-2*q0,  2*q1,  2*q2, -2*q3, 0, 0, 0, 0, 0, 0],
            [dhm_dqw, dhm_dqi, dhm_dqj, dhm_dqk, 0, 0, 0, 0, 0, 0]
        ])

    def update(self):
        v = self.y - self.z
        if v[3] > np.pi:
            v[3] -= 2 * np.pi
        elif v[3] < -np.pi:
            v[3] += 2 * np.pi

        S = self.H @ self.P @ np.matrix.transpose(np.matrix(self.H)) + self.R
        self.K = self.P @ np.matrix.transpose(np.matrix(self.H)) @ np.linalg.inv(S)
        self.x = self.x + self.K @ v
        self.x = np.array(self.x).flatten()
        self.x[0:4] = self.x[0:4] / np.linalg.norm(self.x[0:4])
        self.P = (np.identity(10) - self.K @ self.H) @ self.P


#mat = scipy.io.loadmat('data.mat')
#
#mag = np.array(np.matrix(mat.get('mag')).transpose())
#acc = np.array(np.matrix(mat.get('acc')).transpose())
#gyro = np.array(np.matrix(mat.get('gyro')).transpose())
#gyro = np.array(np.matrix(mat.get('gyro')).transpose())
#time = np.array(mat.get('time')).flatten()
#
#dt = np.diff(time)
#
#ekf = EKF(gyro, acc, mag)
#
#q = list()
#tft = list()
#
#for i in range(len(acc)-1):
#    ekf.update_acc(acc[i])
#    ekf.update_mag(mag[i])
#    ekf.predict(gyro[i], dt[i])
#    ekf.update()
#    q.append(quat2euler(ekf.x[0:4]))
#    tft.append(i)
#
#
#plt.plot(tft, q)
#plt.show()
