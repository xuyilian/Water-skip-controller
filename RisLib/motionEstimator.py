
from scipy.spatial.transform import Rotation as R

import numpy as np


class Estimator:
    def __init__(self):
        self.T = 0.01

        # subsystem vz and z
        self.Q_ZandVZ = [[1, 0], [0, 1]]  # z, vz
        self.R_ZandVZ = [[50, 0], [0, 50]]
        # subsystem vx and vy
        # self.Q_VXandVY = [[1, 0], [0, 1]]  # vx,vy
        # self.R_VXandVY = [[2000, 0], [0, 2000]]

        self.Q_VXandVY = [[1, 0], [0, 1]]  # vx,vy
        self.R_VXandVY = [[5000, 0], [0, 5000]]

        self.X_post_VZ = 0
        self.X_post_Z = 0.0
        self.X_post_VX = 0
        self.X_post_VY = 0

        self.P_post_ZandVZ = [[1, 0], [0, 1]]  # z, vz
        self.P_post_VXandVY = [[0, 0], [0, 0]]

        self.imu_bias_pitch = 0
        self.imu_bias_roll = 0
        self.r_bias = R.from_euler('zyx', [0, self.imu_bias_pitch, self.imu_bias_roll], degrees=True)

    def Estimator_Wall(self, roll, pitch, ax, ay, az, wx, wy, vx_raw, vy_raw, vz, z):
        # input:
        # ax,ay,az,wx,wy,wz from imu,
        # vx_raw,vyraw,vz,z from opti and tof.
        I22 = [[1, 0], [0, 1]]

        # vx_measure = (-vy_raw / 4.0926 - wy) * z
        # vy_measure = (-vx_raw / 4.0926 - wx) * z
        vx_measure = (-vy_raw/4.0926 - wy)*self.X_post_Z
        vy_measure = (-vx_raw/4.0926 - wx)*self.X_post_Z

        # print(round(vx_measure, 2), round(vy_measure, 2))
        # Process Model A,B
        # A
        A_ZandVZ = [[1, self.T], [0, 1]]

        A_VXandVY = [[1, 0], [0, 1]]
        # B
        B_ZandVZ = np.asarray([[0.5 * self.T * self.T], [self.T]])
        B_VXandVY = np.asarray([[self.T, 0], [0, self.T]])

        ## Prediction
        ## Processing
        #  Acc in World
        acc_B = [ax * 9.81, ay * 9.81, az * 9.81]
        r_atti = R.from_euler('zyx', [0, pitch, roll], degrees=True)

        acc_I = r_atti.apply(self.r_bias.apply(acc_B))
        ax_W = acc_I[0]
        ay_W = acc_I[1]
        az_W = acc_I[2] - 9.81

        #  Z and Vz Prediction
        X_ZandVZ = np.matmul(A_ZandVZ, [self.X_post_Z, self.X_post_VZ]) + np.matmul(B_ZandVZ, [az_W])
        P_ZandVZ = np.matmul(A_ZandVZ, np.matmul(self.P_post_ZandVZ, np.transpose(A_ZandVZ))) + self.Q_ZandVZ

        #  VX and VY Prediction
        X_VXandVY = np.matmul(A_VXandVY, [self.X_post_VX, self.X_post_VY]) + np.matmul(B_VXandVY, [ax_W, ay_W])
        P_VXandVY = np.matmul(A_VXandVY, np.matmul(self.P_post_VXandVY, np.transpose(A_VXandVY))) + self.Q_VXandVY

        ## Measurement Model
        #  Z and VZ
        H_ZandVZ = [[1, 0], [0, 1]]
        #  VX and VY
        H_VXandVY = [[1, 0], [0, 1]]

        ## Kalman Gain
        #  Z and VZ  Kalman Gains
        D_ZandVZ = np.matmul(H_ZandVZ, np.matmul(P_ZandVZ, np.transpose(H_ZandVZ))) + self.R_ZandVZ
        K_ZandVZ = np.matmul(P_ZandVZ, np.matmul(np.transpose(H_ZandVZ), np.linalg.inv(D_ZandVZ)))

        #  VX and VY  Kalman Gains
        D_VXandVY = np.matmul(H_VXandVY, np.matmul(P_VXandVY, np.transpose(H_VXandVY))) + self.R_VXandVY
        K_VXandVY = np.matmul(P_VXandVY, np.matmul(np.transpose(H_VXandVY), np.linalg.inv(D_VXandVY)))

        ## Measurement
        #  Z and VZ
        Y_ZandVZ = np.asarray([z, vz])
        Z_ZandVZ = np.asarray([X_ZandVZ[0], X_ZandVZ[1]])

        #  VX and VY
        Y_VXandVY = np.asarray([vx_measure, vy_measure])
        Z_VXandVY = np.asarray([X_VXandVY[0], X_VXandVY[1]])

        ## Updates
        #  Z and VZ Updates
        X_post_ZandVZ = X_ZandVZ + np.matmul(K_ZandVZ, np.subtract(Y_ZandVZ, Z_ZandVZ))
        self.P_post_ZandVZ = np.matmul((np.subtract(I22, np.matmul(K_ZandVZ, H_ZandVZ))), P_ZandVZ)
        self.X_post_Z = X_post_ZandVZ[0]
        self.X_post_VZ = X_post_ZandVZ[1]

        #  VX and VY Updates
        X_post_VXandVY = X_VXandVY + np.matmul(K_VXandVY, np.subtract(Y_VXandVY, Z_VXandVY))
        self.P_post_VXandVY = np.matmul((np.subtract(I22, np.matmul(K_VXandVY, H_VXandVY))), P_VXandVY)
        self.X_post_VX = X_post_VXandVY[0]
        self.X_post_VY = X_post_VXandVY[1]

        return self.X_post_Z, self.X_post_VZ, self.X_post_VX, self.X_post_VY, vx_measure, vy_measure

    def update(self, vx, vy):
        self.X_post_VX = vx
        self.X_post_VY = vy

