import numpy
import viscid.rotation as rot
from numpy import linalg
import math
import mat4py


class Horizontal_4(object):
    # controller based on full second order attitude model and translation model,
    def __init__(self, lambda0=4, lambda1=8, lambda2=8, lambda3=4, c1=-0.2455, c2=8.1983, c3=-0.0471, a1=2.4582, a2=0.0319, a3=-0.0382, a4=0.0478, a5=0.002321, ):
        # control gains
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3
        # parameters
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.C1 = c1
        self.C2 = c2
        self.C3 = c3


        # reference
        self.x_d = 0
        self.y_d = 0
        self.x_d_dot = 0
        self.y_d_dot = 0
        self.x_d_ddot = 0
        self.y_d_ddot = 0
        self.x_d_dddot = 0
        self.y_d_dddot = 0
        self.x_d_ddddot = 0
        self.y_d_ddddot = 0

        # pre-calculation
        self._A52 = self.A5 ** 2
        self._C22 = self.C2 ** 2
        self._C12 = self.C1 ** 2
        self._C13 = self.C1 ** 3
        self._C14 = self.C1 ** 4
        self._C15 = self.C1 ** 5
        self._C32 = self.C3 ** 2
        self._lambda32 = self.lambda3 ** 2
        self._lambda22 = self.lambda2 ** 2
        self._A1C1 = self.A1 * self.C1
        self._A1C2 = self.A1 * self.C2
        self._A2C1 = self.A2 * self.C1
        self._A2C2 = self.A2 * self.C2
        self._A3C1 = self.A3 * self.C1
        self._A3C2 = self.A3 * self.C2
        self._A4C1 = self.A4 * self.C1
        self._A4C2 = self.A4 * self.C2
        self._A5C1 = self.A5 * self.C1
        self._A5C2 = self.A5 * self.C2
        self._C1C3 = self.C1 * self.C3
        self._C2C3 = self.C2 * self.C3
        self._lambda2lambda3 = self.lambda2 * self.lambda3
        self._C2lambda3 = self.C2 * self.lambda3
        self._C2lambda2 = self.C2 * self.lambda2
        self._C3lambda2 = self.C3 * self.lambda2
        self._C3lambda0 = self.C3 * self.lambda0
        self._C3lambda1 = self.C3 * self.lambda1

        self.trajectory_flag = 1
        self.trajectory_end = False
        self._traj_x = None
        self._traj_y = None
        self._traj_xd = None
        self._traj_yd = None
        self._traj_xdd = None
        self._traj_ydd = None
        self._traj_xddd = None
        self._traj_yddd = None
        self._traj_xdddd = None
        self._traj_ydddd = None
        self.traj_len = 0

    def update_parameters(self, c1, c2, c3, a1, a2, a3, a4, a5, ):
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.C1 = c1
        self.C2 = c2
        self.C3 = c3

    def update_gains(self,  lambda0, lambda1, lambda2, lambda3 ):
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3

    def update_reference(self, x_d, y_d, x_d_dot, y_d_dot, x_d_ddot, y_d_ddot, x_d_dddot, y_d_dddot, x_d_ddddot, y_d_ddddot, ):
        self.x_d = x_d
        self.y_d = y_d
        self.x_d_dot = x_d_dot
        self.y_d_dot = y_d_dot
        self.x_d_ddot = x_d_ddot
        self.y_d_ddot = y_d_ddot
        self.x_d_dddot = x_d_dddot
        self.y_d_dddot = y_d_dddot
        self.x_d_ddddot = x_d_ddddot
        self.y_d_ddddot = y_d_ddddot

    def update_error(self, x, y, x_dot, y_dot, xi_x, xi_y, xi_x_dot, xi_y_dot, U_x_dot, U_y_dot, U_x_ddot, U_y_ddot, ):

        denominator_x = self._A52 * self._C22 + self._C14 * self._C32 + 2 * self._C13 * self._C32 * self.lambda3 + 2 * self._C12 * self._C32 * self.lambda2 + self._C12 * self._C32 * self._lambda32 + 2 * self.C1 * self._C32 * self._lambda2lambda3 + self._C32 * self._lambda22
        denominator_y = self._A52 * self._C22 + self._C14 * self._C32 + 2 * self._C13 * self._C32 * self.lambda3 + 2 * self._C12 * self._C32 * self.lambda2 + self._C12 * self._C32 * self._lambda32 + 2 * self.C1 * self._C32 * self._lambda2lambda3 + self._C32 * self._lambda22

        if denominator_x == 0 or denominator_y == 0:
            U_x = 0
            U_y = 0
        else:
            U_x = -(
                        self._A5C2 * self.y_d_ddddot - self._C3lambda2 * self.x_d_ddddot + self._C15 * self.C3 * x_dot - self._C12 * self.C3 * self.x_d_ddddot + self._C32 * U_x_ddot * self.lambda2 - self.C3 * self._lambda22 * self.x_d_ddot + self._C13 * self._C32 * U_x_dot + self._C12 * self._C32 * U_x_ddot + self._C14 * self._C2C3 * xi_y + self._C13 * self._C2C3 * xi_y_dot + self.C1 * self._C32 * U_x_dot * self.lambda2 + self.C1 * self._C32 * U_x_ddot * self.lambda3 + self.A5 * self._C22 * self.lambda2 * xi_x + self.A5 * self._C22 * self.lambda3 * xi_x_dot + self._C12 * self._C3lambda0 * x - self._C12 * self._C3lambda0 * self.x_d + self._C12 * self._C3lambda1 * x_dot + self._C1C3 * self._lambda22 * x_dot + 2 * self._C13 * self._C3lambda2 * x_dot + 2 * self._C14 * self.C3 * self.lambda3 * x_dot - self._C12 * self._C3lambda1 * self.x_d_dot - self._C12 * self._C3lambda2 * self.x_d_ddot - self._C1C3 * self._lambda32 * self.x_d_dddot - self._C12 * self.C3 * self.lambda3 * self.x_d_dddot + self._C2C3 * self._lambda22 * xi_y + self._C32 * U_x_dot * self._lambda2lambda3 + self.A5 * self._C12 * self._C22 * xi_x + self.C1 * self._C32 * U_x_dot * self._lambda32 + 2 * self._C12 * self._C32 * U_x_dot * self.lambda3 - self._A5C2 * self.C3 * U_y_ddot + self._C13 * self.C3 * self._lambda32 * x_dot - self._A5C2 * self.lambda0 * y + self._A5C2 * self.lambda0 * self.y_d - self._A5C2 * self.lambda1 * y_dot + self._A5C2 * self.lambda1 * self.y_d_dot + self._A5C2 * self.lambda2 * self.y_d_ddot + self._A5C2 * self.lambda3 * self.y_d_dddot - self._C1C3 * self.lambda3 * self.x_d_ddddot + self._C3lambda0 * self.lambda2 * x - self._C3lambda0 * self.lambda2 * self.x_d + self._C3lambda1 * self.lambda2 * x_dot - self._C3lambda1 * self.lambda2 * self.x_d_dot - self.C3 * self._lambda2lambda3 * self.x_d_dddot + self.A4 * self.A5 * self._C22 * x_dot + self.A1 * self.A5 * self._C22 * xi_y_dot + self.A2 * self.A5 * self._C22 * xi_x_dot + self.A3 * self.A5 * self._C22 * y_dot + self._A5C1 * self._C22 * xi_x_dot - self.A5 * self._C13 * self.C2 * y_dot - self._A3C2 * self._C3lambda2 * x_dot - self._A1C2 * self._C3lambda2 * xi_x_dot + self._A2C2 * self._C3lambda2 * xi_y_dot - self._A5C1 * self._C2lambda2 * y_dot + self._A4C2 * self._C3lambda2 * y_dot + self.C1 * self._C2C3 * self.lambda2 * xi_y_dot + self._C1C3 * self.lambda0 * self.lambda3 * x - self._C1C3 * self.lambda0 * self.lambda3 * self.x_d + self._C1C3 * self.lambda1 * self.lambda3 * x_dot - self._C1C3 * self.lambda1 * self.lambda3 * self.x_d_dot - self._C1C3 * self._lambda2lambda3 * self.x_d_ddot + self._C2C3 * self._lambda2lambda3 * xi_y_dot - self.A3 * self._C12 * self._C2C3 * x_dot - self.A1 * self._C12 * self._C2C3 * xi_x_dot + self.A2 * self._C12 * self._C2C3 * xi_y_dot + self.A4 * self._C12 * self._C2C3 * y_dot + self._A5C1 * self._C22 * self.lambda3 * xi_x - self.A5 * self._C12 * self._C2lambda3 * y_dot + 2 * self._C12 * self._C2C3 * self.lambda2 * xi_y + 2 * self._C13 * self._C2C3 * self.lambda3 * xi_y + self.C1 * self._C2C3 * self._lambda32 * xi_y_dot + 2 * self._C12 * self._C2C3 * self.lambda3 * xi_y_dot + 2 * self._C12 * self.C3 * self._lambda2lambda3 * x_dot - self._A5C1 * self._C2C3 * U_y_dot + self._C12 * self._C2C3 * self._lambda32 * xi_y - self._A5C2 * self.C3 * U_y_dot * self.lambda3 - self._A3C1 * self._C2C3 * self.lambda3 * x_dot - self._A1C1 * self._C2C3 * self.lambda3 * xi_x_dot + self._A2C1 * self._C2C3 * self.lambda3 * xi_y_dot + self._A4C1 * self._C2C3 * self.lambda3 * y_dot + 2 * self.C1 * self._C2C3 * self._lambda2lambda3 * xi_y) / (
                              denominator_x)
            U_y = (
                              self._A5C2 * self.x_d_ddddot + self._C3lambda2 * self.y_d_ddddot - self._C15 * self.C3 * y_dot + self._C12 * self.C3 * self.y_d_ddddot - self._C32 * U_y_ddot * self.lambda2 + self.C3 * self._lambda22 * self.y_d_ddot - self._C13 * self._C32 * U_y_dot - self._C12 * self._C32 * U_y_ddot + self._C14 * self._C2C3 * xi_x + self._C13 * self._C2C3 * xi_x_dot - self.C1 * self._C32 * U_y_dot * self.lambda2 - self.C1 * self._C32 * U_y_ddot * self.lambda3 - self.A5 * self._C22 * self.lambda2 * xi_y - self.A5 * self._C22 * self.lambda3 * xi_y_dot + self._C2C3 * self._lambda22 * xi_x - self._C12 * self._C3lambda0 * y + self._C12 * self._C3lambda0 * self.y_d - self._C12 * self._C3lambda1 * y_dot - self._C1C3 * self._lambda22 * y_dot - 2 * self._C13 * self._C3lambda2 * y_dot - 2 * self._C14 * self.C3 * self.lambda3 * y_dot + self._C12 * self._C3lambda1 * self.y_d_dot + self._C12 * self._C3lambda2 * self.y_d_ddot + self._C1C3 * self._lambda32 * self.y_d_dddot + self._C12 * self.C3 * self.lambda3 * self.y_d_dddot - self._C32 * U_y_dot * self._lambda2lambda3 - self.A5 * self._C12 * self._C22 * xi_y - self.C1 * self._C32 * U_y_dot * self._lambda32 - 2 * self._C12 * self._C32 * U_y_dot * self.lambda3 - self._A5C2 * self.C3 * U_x_ddot - self._C13 * self.C3 * self._lambda32 * y_dot - self._A5C2 * self.lambda0 * x + self._A5C2 * self.lambda0 * self.x_d - self._A5C2 * self.lambda1 * x_dot + self._A5C2 * self.lambda1 * self.x_d_dot + self._A5C2 * self.lambda2 * self.x_d_ddot + self._A5C2 * self.lambda3 * self.x_d_dddot + self._C1C3 * self.lambda3 * self.y_d_ddddot - self._C3lambda0 * self.lambda2 * y + self._C3lambda0 * self.lambda2 * self.y_d - self._C3lambda1 * self.lambda2 * y_dot + self._C3lambda1 * self.lambda2 * self.y_d_dot + self.C3 * self._lambda2lambda3 * self.y_d_dddot + self.A3 * self.A5 * self._C22 * x_dot + self.A1 * self.A5 * self._C22 * xi_x_dot - self.A2 * self.A5 * self._C22 * xi_y_dot - self.A4 * self.A5 * self._C22 * y_dot - self.A5 * self._C13 * self.C2 * x_dot - self._A5C1 * self._C22 * xi_y_dot - self._A5C1 * self._C2lambda2 * x_dot + self._A4C2 * self._C3lambda2 * x_dot + self._A1C2 * self._C3lambda2 * xi_y_dot + self._A2C2 * self._C3lambda2 * xi_x_dot + self._A3C2 * self._C3lambda2 * y_dot + self.C1 * self._C2C3 * self.lambda2 * xi_x_dot + self._C2C3 * self._lambda2lambda3 * xi_x_dot - self._C1C3 * self.lambda0 * self.lambda3 * y + self._C1C3 * self.lambda0 * self.lambda3 * self.y_d - self._C1C3 * self.lambda1 * self.lambda3 * y_dot + self._C1C3 * self.lambda1 * self.lambda3 * self.y_d_dot + self._C1C3 * self._lambda2lambda3 * self.y_d_ddot + self.A4 * self._C12 * self._C2C3 * x_dot + self.A1 * self._C12 * self._C2C3 * xi_y_dot + self.A2 * self._C12 * self._C2C3 * xi_x_dot + self.A3 * self._C12 * self._C2C3 * y_dot - self.A5 * self._C12 * self._C2lambda3 * x_dot - self._A5C1 * self._C22 * self.lambda3 * xi_y + 2 * self._C12 * self._C2C3 * self.lambda2 * xi_x + 2 * self._C13 * self._C2C3 * self.lambda3 * xi_x + self.C1 * self._C2C3 * self._lambda32 * xi_x_dot + 2 * self._C12 * self._C2C3 * self.lambda3 * xi_x_dot - 2 * self._C12 * self.C3 * self._lambda2lambda3 * y_dot - self._A5C1 * self._C2C3 * U_x_dot + self._C12 * self._C2C3 * self._lambda32 * xi_x - self._A5C2 * self.C3 * U_x_dot * self.lambda3 + self._A4C1 * self._C2C3 * self.lambda3 * x_dot + self._A1C1 * self._C2C3 * self.lambda3 * xi_y_dot + self._A2C1 * self._C2C3 * self.lambda3 * xi_x_dot + self._A3C1 * self._C2C3 * self.lambda3 * y_dot + 2 * self.C1 * self._C2C3 * self._lambda2lambda3 * xi_x) / (
                              denominator_y)
        return U_x, U_y

    def update_error_raw(self, x, y, x_dot, y_dot, xi_x, xi_y, xi_x_dot, xi_y_dot, U_x_dot, U_y_dot, U_x_ddot, U_y_ddot, ):
        denominator_x = self.A5 ** 2 * self.C2 ** 2 + self.C1 ** 4 * self.C3 ** 2 + 2 * self.C1 ** 3 * self.C3 ** 2 * self.lambda3 + 2 * self.C1 ** 2 * self.C3 ** 2 * self.lambda2 + self.C1 ** 2 * self.C3 ** 2 * self.lambda3 ** 2 + 2 * self.C1 * self.C3 ** 2 * self.lambda2 * self.lambda3 + self.C3 ** 2 * self.lambda2 ** 2
        denominator_y = self.A5 ** 2 * self.C2 ** 2 + self.C1 ** 4 * self.C3 ** 2 + 2 * self.C1 ** 3 * self.C3 ** 2 * self.lambda3 + 2 * self.C1 ** 2 * self.C3 ** 2 * self.lambda2 + self.C1 ** 2 * self.C3 ** 2 * self.lambda3 ** 2 + 2 * self.C1 * self.C3 ** 2 * self.lambda2 * self.lambda3 + self.C3 ** 2 * self.lambda2 ** 2
        if denominator_x == 0 or denominator_y == 0:
            U_x = 0
            U_y = 0
        else:
            U_x = -(
                        self.A5 * self.C2 * self.y_d_ddddot - self.C3 * self.lambda2 * self.x_d_ddddot + self.C1 ** 5 * self.C3 * x_dot - self.C1 ** 2 * self.C3 * self.x_d_ddddot + self.C3 ** 2 * U_x_ddot * self.lambda2 - self.C3 * self.lambda2 ** 2 * self.x_d_ddot + self.C1 ** 3 * self.C3 ** 2 * U_x_dot + self.C1 ** 2 * self.C3 ** 2 * U_x_ddot + self.C1 ** 4 * self.C2 * self.C3 * xi_y + self.C1 ** 3 * self.C2 * self.C3 * xi_y_dot + self.C1 * self.C3 ** 2 * U_x_dot * self.lambda2 + self.C1 * self.C3 ** 2 * U_x_ddot * self.lambda3 + self.A5 * self.C2 ** 2 * self.lambda2 * xi_x + self.A5 * self.C2 ** 2 * self.lambda3 * xi_x_dot + self.C1 ** 2 * self.C3 * self.lambda0 * x - self.C1 ** 2 * self.C3 * self.lambda0 * self.x_d + self.C1 ** 2 * self.C3 * self.lambda1 * x_dot + self.C1 * self.C3 * self.lambda2 ** 2 * x_dot + 2 * self.C1 ** 3 * self.C3 * self.lambda2 * x_dot + 2 * self.C1 ** 4 * self.C3 * self.lambda3 * x_dot - self.C1 ** 2 * self.C3 * self.lambda1 * self.x_d_dot - self.C1 ** 2 * self.C3 * self.lambda2 * self.x_d_ddot - self.C1 * self.C3 * self.lambda3 ** 2 * self.x_d_dddot - self.C1 ** 2 * self.C3 * self.lambda3 * self.x_d_dddot + self.C2 * self.C3 * self.lambda2 ** 2 * xi_y + self.C3 ** 2 * U_x_dot * self.lambda2 * self.lambda3 + self.A5 * self.C1 ** 2 * self.C2 ** 2 * xi_x + self.C1 * self.C3 ** 2 * U_x_dot * self.lambda3 ** 2 + 2 * self.C1 ** 2 * self.C3 ** 2 * U_x_dot * self.lambda3 - self.A5 * self.C2 * self.C3 * U_y_ddot + self.C1 ** 3 * self.C3 * self.lambda3 ** 2 * x_dot - self.A5 * self.C2 * self.lambda0 * y + self.A5 * self.C2 * self.lambda0 * self.y_d - self.A5 * self.C2 * self.lambda1 * y_dot + self.A5 * self.C2 * self.lambda1 * self.y_d_dot + self.A5 * self.C2 * self.lambda2 * self.y_d_ddot + self.A5 * self.C2 * self.lambda3 * self.y_d_dddot - self.C1 * self.C3 * self.lambda3 * self.x_d_ddddot + self.C3 * self.lambda0 * self.lambda2 * x - self.C3 * self.lambda0 * self.lambda2 * self.x_d + self.C3 * self.lambda1 * self.lambda2 * x_dot - self.C3 * self.lambda1 * self.lambda2 * self.x_d_dot - self.C3 * self.lambda2 * self.lambda3 * self.x_d_dddot + self.A4 * self.A5 * self.C2 ** 2 * x_dot + self.A1 * self.A5 * self.C2 ** 2 * xi_y_dot + self.A2 * self.A5 * self.C2 ** 2 * xi_x_dot + self.A3 * self.A5 * self.C2 ** 2 * y_dot + self.A5 * self.C1 * self.C2 ** 2 * xi_x_dot - self.A5 * self.C1 ** 3 * self.C2 * y_dot - self.A3 * self.C2 * self.C3 * self.lambda2 * x_dot - self.A1 * self.C2 * self.C3 * self.lambda2 * xi_x_dot + self.A2 * self.C2 * self.C3 * self.lambda2 * xi_y_dot - self.A5 * self.C1 * self.C2 * self.lambda2 * y_dot + self.A4 * self.C2 * self.C3 * self.lambda2 * y_dot + self.C1 * self.C2 * self.C3 * self.lambda2 * xi_y_dot + self.C1 * self.C3 * self.lambda0 * self.lambda3 * x - self.C1 * self.C3 * self.lambda0 * self.lambda3 * self.x_d + self.C1 * self.C3 * self.lambda1 * self.lambda3 * x_dot - self.C1 * self.C3 * self.lambda1 * self.lambda3 * self.x_d_dot - self.C1 * self.C3 * self.lambda2 * self.lambda3 * self.x_d_ddot + self.C2 * self.C3 * self.lambda2 * self.lambda3 * xi_y_dot - self.A3 * self.C1 ** 2 * self.C2 * self.C3 * x_dot - self.A1 * self.C1 ** 2 * self.C2 * self.C3 * xi_x_dot + self.A2 * self.C1 ** 2 * self.C2 * self.C3 * xi_y_dot + self.A4 * self.C1 ** 2 * self.C2 * self.C3 * y_dot + self.A5 * self.C1 * self.C2 ** 2 * self.lambda3 * xi_x - self.A5 * self.C1 ** 2 * self.C2 * self.lambda3 * y_dot + 2 * self.C1 ** 2 * self.C2 * self.C3 * self.lambda2 * xi_y + 2 * self.C1 ** 3 * self.C2 * self.C3 * self.lambda3 * xi_y + self.C1 * self.C2 * self.C3 * self.lambda3 ** 2 * xi_y_dot + 2 * self.C1 ** 2 * self.C2 * self.C3 * self.lambda3 * xi_y_dot + 2 * self.C1 ** 2 * self.C3 * self.lambda2 * self.lambda3 * x_dot - self.A5 * self.C1 * self.C2 * self.C3 * U_y_dot + self.C1 ** 2 * self.C2 * self.C3 * self.lambda3 ** 2 * xi_y - self.A5 * self.C2 * self.C3 * U_y_dot * self.lambda3 - self.A3 * self.C1 * self.C2 * self.C3 * self.lambda3 * x_dot - self.A1 * self.C1 * self.C2 * self.C3 * self.lambda3 * xi_x_dot + self.A2 * self.C1 * self.C2 * self.C3 * self.lambda3 * xi_y_dot + self.A4 * self.C1 * self.C2 * self.C3 * self.lambda3 * y_dot + 2 * self.C1 * self.C2 * self.C3 * self.lambda2 * self.lambda3 * xi_y) / (
                              denominator_x)
            U_y = (
                              self.A5 * self.C2 * self.x_d_ddddot + self.C3 * self.lambda2 * self.y_d_ddddot - self.C1 ** 5 * self.C3 * y_dot + self.C1 ** 2 * self.C3 * self.y_d_ddddot - self.C3 ** 2 * U_y_ddot * self.lambda2 + self.C3 * self.lambda2 ** 2 * self.y_d_ddot - self.C1 ** 3 * self.C3 ** 2 * U_y_dot - self.C1 ** 2 * self.C3 ** 2 * U_y_ddot + self.C1 ** 4 * self.C2 * self.C3 * xi_x + self.C1 ** 3 * self.C2 * self.C3 * xi_x_dot - self.C1 * self.C3 ** 2 * U_y_dot * self.lambda2 - self.C1 * self.C3 ** 2 * U_y_ddot * self.lambda3 - self.A5 * self.C2 ** 2 * self.lambda2 * xi_y - self.A5 * self.C2 ** 2 * self.lambda3 * xi_y_dot + self.C2 * self.C3 * self.lambda2 ** 2 * xi_x - self.C1 ** 2 * self.C3 * self.lambda0 * y + self.C1 ** 2 * self.C3 * self.lambda0 * self.y_d - self.C1 ** 2 * self.C3 * self.lambda1 * y_dot - self.C1 * self.C3 * self.lambda2 ** 2 * y_dot - 2 * self.C1 ** 3 * self.C3 * self.lambda2 * y_dot - 2 * self.C1 ** 4 * self.C3 * self.lambda3 * y_dot + self.C1 ** 2 * self.C3 * self.lambda1 * self.y_d_dot + self.C1 ** 2 * self.C3 * self.lambda2 * self.y_d_ddot + self.C1 * self.C3 * self.lambda3 ** 2 * self.y_d_dddot + self.C1 ** 2 * self.C3 * self.lambda3 * self.y_d_dddot - self.C3 ** 2 * U_y_dot * self.lambda2 * self.lambda3 - self.A5 * self.C1 ** 2 * self.C2 ** 2 * xi_y - self.C1 * self.C3 ** 2 * U_y_dot * self.lambda3 ** 2 - 2 * self.C1 ** 2 * self.C3 ** 2 * U_y_dot * self.lambda3 - self.A5 * self.C2 * self.C3 * U_x_ddot - self.C1 ** 3 * self.C3 * self.lambda3 ** 2 * y_dot - self.A5 * self.C2 * self.lambda0 * x + self.A5 * self.C2 * self.lambda0 * self.x_d - self.A5 * self.C2 * self.lambda1 * x_dot + self.A5 * self.C2 * self.lambda1 * self.x_d_dot + self.A5 * self.C2 * self.lambda2 * self.x_d_ddot + self.A5 * self.C2 * self.lambda3 * self.x_d_dddot + self.C1 * self.C3 * self.lambda3 * self.y_d_ddddot - self.C3 * self.lambda0 * self.lambda2 * y + self.C3 * self.lambda0 * self.lambda2 * self.y_d - self.C3 * self.lambda1 * self.lambda2 * y_dot + self.C3 * self.lambda1 * self.lambda2 * self.y_d_dot + self.C3 * self.lambda2 * self.lambda3 * self.y_d_dddot + self.A3 * self.A5 * self.C2 ** 2 * x_dot + self.A1 * self.A5 * self.C2 ** 2 * xi_x_dot - self.A2 * self.A5 * self.C2 ** 2 * xi_y_dot - self.A4 * self.A5 * self.C2 ** 2 * y_dot - self.A5 * self.C1 ** 3 * self.C2 * x_dot - self.A5 * self.C1 * self.C2 ** 2 * xi_y_dot - self.A5 * self.C1 * self.C2 * self.lambda2 * x_dot + self.A4 * self.C2 * self.C3 * self.lambda2 * x_dot + self.A1 * self.C2 * self.C3 * self.lambda2 * xi_y_dot + self.A2 * self.C2 * self.C3 * self.lambda2 * xi_x_dot + self.A3 * self.C2 * self.C3 * self.lambda2 * y_dot + self.C1 * self.C2 * self.C3 * self.lambda2 * xi_x_dot + self.C2 * self.C3 * self.lambda2 * self.lambda3 * xi_x_dot - self.C1 * self.C3 * self.lambda0 * self.lambda3 * y + self.C1 * self.C3 * self.lambda0 * self.lambda3 * self.y_d - self.C1 * self.C3 * self.lambda1 * self.lambda3 * y_dot + self.C1 * self.C3 * self.lambda1 * self.lambda3 * self.y_d_dot + self.C1 * self.C3 * self.lambda2 * self.lambda3 * self.y_d_ddot + self.A4 * self.C1 ** 2 * self.C2 * self.C3 * x_dot + self.A1 * self.C1 ** 2 * self.C2 * self.C3 * xi_y_dot + self.A2 * self.C1 ** 2 * self.C2 * self.C3 * xi_x_dot + self.A3 * self.C1 ** 2 * self.C2 * self.C3 * y_dot - self.A5 * self.C1 ** 2 * self.C2 * self.lambda3 * x_dot - self.A5 * self.C1 * self.C2 ** 2 * self.lambda3 * xi_y + 2 * self.C1 ** 2 * self.C2 * self.C3 * self.lambda2 * xi_x + 2 * self.C1 ** 3 * self.C2 * self.C3 * self.lambda3 * xi_x + self.C1 * self.C2 * self.C3 * self.lambda3 ** 2 * xi_x_dot + 2 * self.C1 ** 2 * self.C2 * self.C3 * self.lambda3 * xi_x_dot - 2 * self.C1 ** 2 * self.C3 * self.lambda2 * self.lambda3 * y_dot - self.A5 * self.C1 * self.C2 * self.C3 * U_x_dot + self.C1 ** 2 * self.C2 * self.C3 * self.lambda3 ** 2 * xi_x - self.A5 * self.C2 * self.C3 * U_x_dot * self.lambda3 + self.A4 * self.C1 * self.C2 * self.C3 * self.lambda3 * x_dot + self.A1 * self.C1 * self.C2 * self.C3 * self.lambda3 * xi_y_dot + self.A2 * self.C1 * self.C2 * self.C3 * self.lambda3 * xi_x_dot + self.A3 * self.C1 * self.C2 * self.C3 * self.lambda3 * y_dot + 2 * self.C1 * self.C2 * self.C3 * self.lambda2 * self.lambda3 * xi_x) / (
                              denominator_y)

        return U_x, U_y

    def load_trajectory(self, file_path):
        MAT_Trajectory = mat4py.loadmat(file_path)
        self._traj_x = MAT_Trajectory['x']
        self._traj_y = MAT_Trajectory['y']
        # self.traj_z = MAT_Trajectory['z']
        self._traj_xd = MAT_Trajectory['xd']
        self._traj_yd = MAT_Trajectory['yd']
        # self.traj_zd = MAT_Trajectory['zd']
        self._traj_xdd = MAT_Trajectory['xdd']
        self._traj_ydd = MAT_Trajectory['ydd']
        # self.traj_zdd = MAT_Trajectory['zdd']
        self._traj_xddd = MAT_Trajectory['xddd']
        self._traj_yddd = MAT_Trajectory['yddd']
        # self.traj_zddd = MAT_Trajectory['zddd']
        self._traj_xdddd = MAT_Trajectory['xdddd']
        self._traj_ydddd = MAT_Trajectory['ydddd']
        # self.traj_zdddd = MAT_Trajectory['zdddd']
        self.traj_len = len(self._traj_x)

    def update_desired_position_trajectory(self):

        self.trajectory_flag = self.trajectory_flag + 1
        if self.trajectory_flag > self.traj_len - 3:
            self.trajectory_end = True
        else:
            self.x_d = self._traj_x[self.trajectory_flag]
            self.y_d = self._traj_y[self.trajectory_flag]
            self.x_d_dot = self._traj_xd[self.trajectory_flag]
            self.y_d_dot = self._traj_yd[self.trajectory_flag]
            self.x_d_ddot = self._traj_xdd[self.trajectory_flag]
            self.y_d_ddot = self._traj_ydd[self.trajectory_flag]
            self.x_d_dddot = self._traj_xddd[self.trajectory_flag]
            self.y_d_dddot = self._traj_yddd[self.trajectory_flag]
            self.x_d_ddddot = self._traj_xdddd[self.trajectory_flag]
            self.y_d_ddddot = self._traj_ydddd[self.trajectory_flag]
            # self.x_d_dddot = 0
            # self.y_d_dddot = 0
            # self.x_d_ddddot = 0
            # self.y_d_ddddot = 0




class Horizontal_3(object):
    # controller based on full second order attitude model and translation model,
    def __init__(self, lambda0, lambda1, lambda2, lambda3, c1, c2, c3, c4, a1, a2, a3, a4, a5, delta_t):
        # control gains
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3
        # parameters
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.C1 = c1
        self.C2 = c2
        self.C3 = c3
        self.C4 = c4

        self._delta_t = delta_t
        self.inv_delta_t = 1 / self._delta_t
        # reference
        self.p_desired = numpy.array([[0, 0]]).T
        self.p_d_desired = numpy.array([[0, 0]]).T
        self.p_dd_desired = numpy.array([[0, 0]]).T
        self.p_ddd_desired = numpy.array([[0, 0]]).T
        self.p_dddd_desired = numpy.array([[0, 0]]).T

        self._I_matrix = numpy.array([[0, 1], [-1, 0]])
        self._R1 = numpy.array([[math.cos(self.C4), -math.sin(self.C4)], [math.sin(self.C4), math.cos(self.C4)]])
        self._R2 = numpy.array([[math.cos(self.A5), -math.sin(self.A5)], [math.sin(self.A5), math.cos(self.A5)]])
        self._C1C1 = self.C1 * self.C1
        self._C1C3 = self.C1 * self.C3
        self._C3R1 = self.C3 * self._R1
        self._delta_t_delta_t = self._delta_t * self._delta_t
        self.inv_delta_t_delta_t = 1 / self._delta_t_delta_t

        self.uz1 = numpy.array([[0, 0]]).T
        self.uz2 = numpy.array([[0, 0]]).T

    def update_parameters(self, c1, c2, a1, a2, a3, a4, a5, ):
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.C1 = c1
        self.C2 = c2

    def update_gains(self,  lambda0, lambda1, lambda2, lambda3 ):
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3

    def update_reference(self, p, p_d, p_dd, p_ddd, p_dddd):
        # p's must be 2x1 matrix
        self.p_desired = p
        self.p_d_desired = p_d
        self.p_dd_desired = p_dd
        self.p_ddd_desired = p_ddd
        self.p_dddd_desired = p_dddd

    def update_error(self, p, p_dot, xi, xi_dot):

        C1p_dot = self.C1 * p_dot
        C2Ixi = self.C2 * self._I_matrix.dot(xi)
        C2Ixi_dot = self.C2 * self._I_matrix.dot(xi_dot)

        first_term = - self.lambda0 * (p - self.p_desired) \
                     - self.lambda1 * (p_dot - self.p_d_desired) \
                     - self.lambda2 * (C1p_dot + C2Ixi - self.p_dd_desired) \
                     - self.lambda3 * (self.C1 * C1p_dot + self.C1 * C2Ixi + C2Ixi_dot - self.p_ddd_desired) \
                     - self._C1C1 * C1p_dot - self._C1C1 * C2Ixi - self.C1 * C2Ixi_dot + self.C2 * self.A1 * xi_dot \
                     - self.A2 * C2Ixi_dot + self.p_dddd_desired - self.C2 * self.A3 * self._I_matrix.dot(p_dot) \
                     + self._C1C3 * self.inv_delta_t * self._R1.dot(self.uz1) \
                     + self.lambda3 * self.inv_delta_t * self._C3R1.dot(self.uz1) \
                     + 2 * self.inv_delta_t_delta_t * self._C3R1.dot(self.uz1) \
                     - self.inv_delta_t_delta_t * self._C3R1.dot(self.uz2)

        second_term = + self.C2 * self.A4 * self._I_matrix.dot(self._R2) \
                      + self.inv_delta_t_delta_t * self._C3R1 \
                      + (self._C1C3 * self.inv_delta_t) * self._R1 \
                      + self._C1C1 * self._C3R1 \
                      + self.lambda2 * self._C3R1 \
                      + self.lambda3 * self._C1C3 * self._R1 \
                      + (self.lambda3 * self.inv_delta_t * self._C3R1)

        u = numpy.linalg.inv(second_term).dot(first_term)

        # norm_u = math.sqrt(u[0] * u[0] + u[1] * u[1])
        # if norm_u > 40:
        #     u = u / norm_u

        self.uz2 = self.uz1
        self.uz1 = u
        return u


class Horizontal_2(object):
    # controller based on second order attitude model, but the horizontal actuator forces are ignored
    def __init__(self, lambda0, lambda1, lambda2, lambda3, c1, c2, a1, a2, a3, a4, a5, ):
        # control gains
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3
        # parameters
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.C1 = c1
        self.C2 = c2

        # reference
        self.p_desired = numpy.array([[0, 0]]).T
        self.p_d_desired = numpy.array([[0, 0]]).T
        self.p_dd_desired = numpy.array([[0, 0]]).T
        self.p_ddd_desired = numpy.array([[0, 0]]).T
        self.p_dddd_desired = numpy.array([[0, 0]]).T

        self.I_matrix = numpy.array([[0, 1], [-1, 0]])

    def update_parameters(self, c1, c2, a1, a2, a3, a4, a5, ):
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.C1 = c1
        self.C2 = c2

    def update_gains(self,  lambda0, lambda1, lambda2, lambda3 ):
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3

    def update_reference(self, p, p_d, p_dd, p_ddd, p_dddd):
        # p's must be 2x1 matrix
        self.p_desired = p
        self.p_d_desired = p_d
        self.p_dd_desired = p_dd
        self.p_ddd_desired = p_ddd
        self.p_dddd_desired = p_dddd

    def update_error(self, p, p_dot, xi, xi_dot, ):
        R = numpy.array([[math.cos(self.A5), -math.sin(self.A5)], [math.sin(self.A5), math.cos(self.A5)]])
        first_term = - self.lambda3 * (pow(self.C1, 2) * p_dot
                                       + self.C1 * self.C2 * self.I_matrix.dot(xi)
                                       + self.C2 * self.I_matrix.dot(xi_dot) - self.p_ddd_desired)\
                     - self.lambda2 * (self.C1 * p_dot + self.C2 * self.I_matrix.dot(xi) - self.p_dd_desired) \
                     - self.lambda1 * (p_dot - self.p_d_desired)\
                     - self.lambda0 * (p - self.p_desired)\
                     - pow(self.C1, 3) * p_dot - pow(self.C1, 2) * self.C2 * self.I_matrix.dot(xi)\
                     - self.C1 * self.C2 * self.I_matrix.dot(xi_dot) + self.C2 * self.A1 * xi_dot \
                     - self.A2 * self.C2 * self.I_matrix.dot(xi_dot) - self.A3 * self.C2 * self.I_matrix.dot(p_dot) + self.p_dddd_desired
        second_term = self.C2 * self.I_matrix.dot(self.A4 * R)
        u = numpy.linalg.inv(second_term).dot(first_term)
        return u


class Horizontal(object):
    # controller based on first order attitude model
    def __init__(self, lambda0, lambda1, lambda2, a1, a2, k1, k2, ):
        # control gains
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        # parameters
        self.A1 = a1
        self.A2 = a2
        self.K1 = k1
        self.K2 = k2
        # reference
        self.p_desired = numpy.array([[0, 0]]).T
        self.p_d_desired = numpy.array([[0, 0]]).T
        self.p_dd_desired = numpy.array([[0, 0]]).T
        self.p_ddd_desired = numpy.array([[0, 0]]).T

        self.I_matrix = numpy.array([[0, 1], [-1, 0]])

    def update_parameters(self, a1, a2, k1, k2, ):
        self.A1 = a1
        self.A2 = a2
        self.K1 = k1
        self.K2 = k2

    def update_gains(self, lambda0, lambda1, lambda2, ):
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2

    def update_reference(self, p, p_d, p_dd, p_ddd):
        # p's must be 2x1 matrix
        self.p_desired = p
        self.p_d_desired = p_d
        self.p_dd_desired = p_dd
        self.p_ddd_desired = p_ddd

    def update_error(self, p, p_dot, xi, ):
        first_term = - (self.A1 * p_dot + self.A2 * self.I_matrix.dot(xi) - self.p_dd_desired) * self.lambda2 \
                     - self.lambda1 * (p_dot - self.p_d_desired) \
                     - self.lambda0 * (p - self.p_desired) \
                     - self.A1 * self.A1 * p_dot \
                     - self.A1 * self.A2 * self.I_matrix.dot(xi) \
                     - self.A2 * self.I_matrix.dot(self.K1).dot(p_dot) + self.p_ddd_desired
        second_term = self.A2 * self.I_matrix.dot(self.K2)
        u = numpy.linalg.inv(second_term).dot(first_term)
        return u


class Altitude(object):
    def __init__(self, lambda0=8, lambda1=12, lambda2=6,
                 a1=0, a2=0, a3=0, a4=0, a5=0, a6=0, a7=0, a8=0, sample_time=0.01):
        self.dt = sample_time
        # control gains
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        # model parameters
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.A6 = a6
        self.A7 = a7
        self.A8 = a8
        # reference
        self.z_desired = 0
        self.z_desired_dot = 0
        self.z_desired_ddot = 0
        self.z_desired_dddot = 0

        self.f = 0

        self.traj_z = None
        self.traj_zd = None
        self.traj_zdd = None
        self.traj_zddd = None
        self.traj_len = 0
        self.trajectory_flag = 1
        self.trajectory_end = False

    def update_parameters(self, a1, a2, a3, a4, a5, a6, a7, a8, ):
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.A6 = a6
        self.A7 = a7
        self.A8 = a8

    def update_gains(self,  lambda0, lambda1, lambda2,):
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2

    def update_reference(self, z_desired, z_desired_dot, z_desired_ddot, z_desired_dddot):
        self.z_desired = z_desired
        self.z_desired_dot = z_desired_dot
        self.z_desired_ddot = z_desired_ddot
        self.z_desired_dddot = z_desired_dddot

    def update_error(self, omega, z, z_dot, z_ddot):
        f_delay = self.f
        temp = (self.A2 * 2 * omega * numpy.sign(omega) * self.A7 + self.A3 / self.dt + self.lambda2 * self.A3)
        if temp == 0:
            f = 0
        else:
            f = (self.z_desired_dddot - self.lambda1 * (z_dot - self.z_desired_dot)
                 - self.lambda0 * (z - self.z_desired)
                 - self.lambda2 * (self.A1 * z_dot + self.A2 * omega * abs(omega) + self.A4 - self.z_desired_ddot)
                 - self.A1 * z_ddot - self.A2 * 2.0 * omega * numpy.sign(omega)
                 * (self.A5 * z_dot + self.A6 * omega * abs(omega) + self.A8)
                 + self.A3 * f_delay / self.dt) / temp
        self.f = f
        return self.f

    def load_trajectory(self, file_path):
        MAT_Trajectory = mat4py.loadmat(file_path)
        self.traj_z = MAT_Trajectory['z']
        self.traj_zd = MAT_Trajectory['zd']
        self.traj_zdd = MAT_Trajectory['zdd']
        self.traj_zddd = MAT_Trajectory['zddd']
        self.traj_len = len(self.traj_z)

    def update_desired_position_trajectory(self):

        self.trajectory_flag = self.trajectory_flag + 1
        if self.trajectory_flag > self.traj_len - 3:
            self.trajectory_end = True
        else:
            self.z_desired = self.traj_z[self.trajectory_flag]
            self.z_desired_dot = self.traj_zd[self.trajectory_flag]
            self.z_desired_ddot = self.traj_zdd[self.trajectory_flag]
            self.z_desired_dddot = self.traj_zddd[self.trajectory_flag]


class AdaptiveAttitudeFilter(object):
    def __init__(self, gain):
        self._rotm_delayed = rot.quat2rotm([1, 0, 0, 0])
        self._axang_offset = rot.rotm2axang(self._rotm_delayed)
        self.gain = gain
        self._gain_2 = 1 - gain

    def step(self, qw, qx, qy, qz):
        inv_rotm = numpy.linalg.inv(self._rotm_delayed)
        rotm = rot.quat2rotm([qw, qx, qy, qz])
        delta_rotm = inv_rotm.dot(rotm)
        axang = rot.rotm2axang(delta_rotm, bad_matrix='ignore')
        self._rotm_delayed = rotm
        axang_offset_temp = numpy.dot(self._axang_offset, self.gain) + numpy.dot(axang, self._gain_2)
        self._axang_offset = axang_offset_temp / linalg.norm(axang_offset_temp)
        return self._axang_offset

