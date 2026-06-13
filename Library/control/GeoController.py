import numpy as np
import os
import mat4py


class YawController(object):
    def __init__(self, sample_time, kp_yaw, kd_yaw, ki_yaw):
        self.sample_time = sample_time
        self.Kp_yaw = kp_yaw
        self.Kd_yaw = kd_yaw
        self.Ki_yaw = ki_yaw
        self.Ki_yaw_temp = 0
        self.desired_yaw = 0
        self.desired_yaw_rate = 0
        self.error_yaw = 0
        self.error_yaw_int = 0

    def update_error(self, dp1):
        delayed_error_yaw = self.error_yaw
        self.error_yaw = self.desired_yaw - dp1.angleY
        self.error_yaw_int = self.error_yaw_int + ((self.error_yaw + delayed_error_yaw) * self.sample_time / 2)
        u_yaw = round(self.Kp_yaw * self.error_yaw +
                      self.Kd_yaw * (self.desired_yaw_rate - dp1.angleY_d) +
                      self.Ki_yaw_temp * self.error_yaw_int)
        return u_yaw

    def update_desired_state(self,desired_yaw, desired_yaw_rate, ):
        self.desired_yaw = desired_yaw
        self.desired_yaw_rate = desired_yaw_rate

    def integrator_enable(self):
        self.Ki_yaw_temp = self.Ki_yaw

def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x
    
class Controller(object):
    def __init__(self, sample_time, delayed_angle,
                 lambda_i, lambda0, lambda1, lambda2, lambda3,
                 c3, c4, c5,
                 kp_z, ki_z, kd_z, constant_z, ):
        self.sample_time = sample_time

        # ---- controller init
        self.delayed_angle = delayed_angle

        self.lambda_i_temp = 0
        self.lambda_i = lambda_i
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3

        self.C3 = c3
        self.C4 = c4
        self.C5 = c5

        self.Kp_Z = kp_z
        self.Ki_Z_temp = 0
        self.Ki_Z = ki_z
        self.Kd_Z = kd_z
        self.constant_Z = constant_z

        self.IError_Z = 0
        self.IError_X = 0
        self.IError_Y = 0

        self.Delayed_Error_Z = 0
        self.Delayed_Error_X = 0
        self.Delayed_Error_Y = 0

        self.Desired_X = 0
        self.Desired_Y = 0
        self.Desired_Z = 0

        self.desired_x_d = 0
        self.desired_y_d = 0

        self.Error_Z = 0
        self.Error_X = 0
        self.Error_Y = 0

        self.P_d = np.array([[0], [0]])
        self.P_d_dot = np.array([[0], [0]])
        self.P_d_2dot = np.array([[0], [0]])
        self.P_d_3dot = np.array([[0], [0]])
        self.P_d_4dot = np.array([[0], [0]])

        self.trajectory_flag = 0
        self.traj_len = 0
        self.x = None
        self.y = None
        self.z = None
        self.xd = None
        self.yd = None
        self.zd = None
        self.xdd = None
        self.ydd = None
        self.zdd = None
        self.xddd = None
        self.yddd = None
        self.zddd = None
        self.xdddd = None
        self.ydddd = None
        self.zdddd = None

    def update_desired_position(self, desired_x, desired_y, desired_z, dt=None):
        delayed_desired_x = self.Desired_X
        delayed_desired_y = self.Desired_Y

        if dt is None:
            dt = self.sample_time

        self.Desired_X = desired_x
        self.Desired_Y = desired_y
        self.Desired_Z = desired_z

        self.desired_x_d = (desired_x - delayed_desired_x) / dt
        self.desired_y_d = (desired_y - delayed_desired_y) / dt

        self.P_d = np.array([[desired_x], [desired_y]])
        self.P_d_dot = np.array([[self.desired_x_d], [self.desired_y_d]])
        self.P_d_2dot = np.array([[0], [0]])
        self.P_d_3dot = np.array([[0], [0]])
        self.P_d_4dot = np.array([[0], [0]])

    def update_error(self, dp1, dt=None):
        delayed_error_z = self.Error_Z
        delayed_error_x = self.Error_X
        delayed_error_y = self.Error_Y

        if dt is None:
            dt = self.sample_time

        self.Error_Z = self.Desired_Z - dp1.Z_F
        self.Error_X = self.Desired_X - dp1.X_F
        self.Error_Y = self.Desired_Y - dp1.Y_F

        d_error_z = (self.Error_Z - delayed_error_z) / dt
        # Tustin Approximation
        self.IError_Z = self.IError_Z + ((self.Error_Z + delayed_error_z) * dt / 2)
        self.IError_X = self.IError_X + ((self.Error_X + delayed_error_x) * dt / 2)
        self.IError_Y = self.IError_Y + ((self.Error_Y + delayed_error_y) * dt / 2)
        u_int_z = self.Ki_Z_temp * self.IError_Z
        # altitude controller
        u_z_output = round(saturation((self.Kp_Z * self.Error_Z + u_int_z + self.Kd_Z * d_error_z) + self.constant_Z, 65000,2000))

        # attitude and position controller
        a_temp = np.array([[0, -1], [1, 0]]) * self.C4 / self.C3
        a_temp2 = np.array([[0, -self.C3], [self.C3, 0]])
        a_temp3 = np.array([[0, self.C5 * dp1.angleY_d], [-self.C5 * dp1.angleY_d, 0]])

        b_tempi = self.lambda_i_temp * (np.array([[self.IError_X], [self.IError_Y]]))
        b_temp1 = self.lambda0 * (- np.array([[dp1.X_F], [dp1.Y_F]]) + self.P_d)
        b_temp2 = self.lambda1 * (- np.array([[dp1.X_F_d], [dp1.Y_F_d]]) + self.P_d_dot)
        b_temp3 = self.lambda2 * (a_temp2.dot(np.array([[-dp1.R23_F], [dp1.R13_F]])) + self.P_d_2dot)
        b_temp4 = self.lambda3 * (a_temp2.dot(np.array([[-dp1.R23_F_d], [dp1.R13_F_d]])) + self.P_d_3dot)
        b_temp5 = self.P_d_4dot

        c_temp = a_temp.dot(b_tempi + b_temp1 + b_temp2 + b_temp3 + b_temp4 + b_temp5) + a_temp3.dot(
            np.array([[-dp1.R23_F_d], [dp1.R13_F_d]]))

        u_x = c_temp[1].astype(float)
        u_y = -c_temp[0].astype(float)
        u_x_output = u_x[0]
        u_y_output = u_y[0]

        return u_x_output, u_y_output, u_z_output

    def integrator_enable(self):
        self.Ki_Z_temp = self.Ki_Z
        self.lambda_i_temp = self.lambda_i

    def position_control_disable(self):
        self.lambda0 = 0
        self.lambda1 = 0

    def position_control_enable(self, lambda0, lambda1):
        self.lambda0 = lambda0
        self.lambda1 = lambda1

    def position_altitude_control_enable(self, ):
        self.Kp_Z = 10000
        self.Kd_Z = 9000

    def load_trajectory(self):
        MAT_Trajectory = mat4py.loadmat(
            os.path.abspath(os.path.join(os.getcwd(), "..")) + '/Bicopter/DataExchange/Trajecotries/T1.mat')
        self.x = MAT_Trajectory['x']
        self.y = MAT_Trajectory['y']
        self.z = MAT_Trajectory['z']
        self.xd = MAT_Trajectory['xd']
        self.yd = MAT_Trajectory['yd']
        self.zd = MAT_Trajectory['zd']
        self.xdd = MAT_Trajectory['xdd']
        self.ydd = MAT_Trajectory['ydd']
        self.zdd = MAT_Trajectory['zdd']
        self.xddd = MAT_Trajectory['xddd']
        self.yddd = MAT_Trajectory['yddd']
        self.zddd = MAT_Trajectory['zddd']
        self.xdddd = MAT_Trajectory['xdddd']
        self.ydddd = MAT_Trajectory['ydddd']
        self.zdddd = MAT_Trajectory['zdddd']
        self.traj_len = len(self.x)

    def update_desired_position_trajectory(self):

        self.trajectory_flag = self.trajectory_flag+1
        if self.trajectory_flag > self.traj_len - 3:
            pass
        else:
            self.Desired_X = self.x[self.trajectory_flag]
            self.Desired_Y = self.y[self.trajectory_flag]
            self.Desired_Z = self.z[self.trajectory_flag]

            self.desired_x_d = self.xd[self.trajectory_flag]
            self.desired_y_d = self.yd[self.trajectory_flag]

            self.P_d = np.array([[self.x[self.trajectory_flag]], [self.y[self.trajectory_flag]]])
            self.P_d_dot = np.array([[self.xd[self.trajectory_flag]], [self.yd[self.trajectory_flag]]])
            self.P_d_2dot = np.array([[self.xdd[self.trajectory_flag]], [self.ydd[self.trajectory_flag]]])
            self.P_d_3dot = np.array([[self.xddd[self.trajectory_flag]], [self.yddd[self.trajectory_flag]]])
            self.P_d_4dot = np.array([[self.xdddd[self.trajectory_flag]], [self.ydddd[self.trajectory_flag]]])
