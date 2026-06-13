import numpy as np
from transforms3d.quaternions import quat2mat


def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x


def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)


class Differentiator:
    def __init__(self, ):
        self.t_delay = 0
        self.data_delay = -1

    def step(self, data_now, abstime):
        dt = abstime - self.t_delay
        if dt == 0:
            self.t_delay = abstime
            return 0
        else:
            data_rate = (data_now - self.data_delay) / (abstime - self.t_delay)
            self.t_delay = abstime
            self.data_delay = data_now
            return data_rate


class RiseDetect:
    def __init__(self, ):
        self.flag = False
        self.flag_old = False

    def step(self, input):
        self.flag_old = self.flag
        self.flag = input

        if self.flag and not self.flag_old:
            return True
        else:
            return False


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def quatinv(quaternion):
    w1, x1, y1, z1 = quaternion
    q_norm = 1 / (w1 * w1 + x1 * x1 + y1 * y1 + z1 * z1)
    return np.array([w1 * q_norm, -x1 * q_norm, -y1 * q_norm, -z1 * q_norm], dtype=np.float64)
    pass


class ThrustModel:
    def __init__(self, c_0, c_1, c_2, gain_0, gain_1, gain_2):
        self.c_0 = c_0
        self.c_1 = c_1
        self.c_2 = c_2
        self.g = 9.81
        self.gain_0 = gain_0
        self.gain_1 = gain_1
        self.gain_2 = gain_2
        self.z_ddot_hat = 0

    def thrust_prediction(self, thrust_commanded, z_dot):
        self.z_ddot_hat = self.c_0 + self.c_1 * thrust_commanded + self.c_2 * z_dot - self.g
        return self.z_ddot_hat

    def step(self, z_ddot, thrust_commanded, z_dot):
        error = self.thrust_prediction(thrust_commanded, z_dot) - z_ddot
        self.c_0 = self.c_0 - error * self.gain_0
        self.c_1 = self.c_1 - error * self.gain_1 * thrust_commanded
        self.c_2 = self.c_2 - error * self.gain_2 * z_dot

    def generate_acc(self, acc_desired, z_dot):
        return (acc_desired - self.c_0 - self.c_2 * z_dot + self.g) / self.c_1


def get_hrotm(qw, qx, qy, qz, x, y, z):
    rotm = quat2mat([qw, qx, qy, qz])
    P = np.array([[x, y, z]]).T
    HB = np.array([[0, 0, 0, 1]])
    temp = np.concatenate((rotm, P), axis=1)
    hrotm = np.concatenate((temp, HB), axis=0)
    return hrotm
