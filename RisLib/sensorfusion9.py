import numpy as np
import math


def quatinv(quaternion):
    w1, x1, y1, z1 = quaternion
    q_norm = 1/(w1*w1 + x1*x1 + y1*y1 + z1*z1)
    return np.array([w1*q_norm, -x1*q_norm, -y1*q_norm, -z1*q_norm], dtype=np.float64)
    pass


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


class SensorFusion9:
    def __init__(self, d, ):
        self.d = d
        self.time_delay = 0
        self.quat_comp = np.array([1, 0, 0, 0])

    def step(self, omega, acc, yaw, time_now):
        acc_offset_predicted = np.cross(omega, np.cross(omega, self.d))
        acc_body = acc - acc_offset_predicted
        temp1 = np.cross(acc_body, [0, 0, 1])
        temp2 = np.sqrt(np.sum(acc_body*acc_body)) + np.dot(acc_body, [0, 0, 1])
        temp3 = np.concatenate(([temp2], temp1))
        quat_acc = temp3/np.sqrt(np.sum(temp3*temp3))
        quat_yaw_mocap = np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
        quat_acc_mocap = quaternion_multiply(quat_yaw_mocap, quat_acc)

        dt = time_now - self.time_delay
        W = [[0, -omega[0], -omega[1], -omega[2]],
             [omega[0], 0, omega[2], -omega[1]],
             [omega[1], -omega[2], 0, omega[0]],
             [omega[2], omega[1], -omega[0], 0]]

        quat_dot = 0.5*np.matmul(W, self.quat_comp.T)
        temp4 = self.quat_comp + quat_dot * dt
        quat_comp_ = temp4/np.sqrt(np.sum(temp4*temp4))
        quat_diff = quaternion_multiply(quatinv(quat_comp_), quat_acc_mocap)
        quat_diff[0] = quat_diff[0] * 500
        temp5 = quaternion_multiply(quat_comp_, quat_diff)
        self.quat_comp = temp5/np.sqrt(np.sum(temp5*temp5))
        self.time_delay = time_now
        return self.quat_comp

    def reset_quat(self, qw, qx, qy, qz):
        self.quat_comp = np.array([qw, qx, qy, qz])

