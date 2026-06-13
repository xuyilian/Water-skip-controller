# standard pid position controller for crazyflie


class Controller(object):
    def __init__(self, sample_time,
                 k_p_x, k_i_x, k_d_x,
                 k_p_y, k_i_y, k_d_y,
                 k_p_z, k_i_z, k_d_z,
                 k_p_yaw, k_i_yaw, k_d_yaw,):
        self.sample_time = sample_time

        self.k_p_x = k_p_x
        self.k_i_x = k_i_x
        self.k_i_x_temp = 0
        self.k_d_x = k_d_x

        self.k_p_y = k_p_y
        self.k_i_y = k_i_y
        self.k_i_y_temp = 0
        self.k_d_y = k_d_y

        self.k_p_z = k_p_z
        self.k_i_z = k_i_z
        self.k_i_z_temp = 0
        self.k_d_z = k_d_z

        self.k_p_yaw = k_p_yaw
        self.k_i_yaw = k_i_yaw
        self.k_i_yaw_temp = 0
        self.k_d_yaw = k_d_yaw

        self.Desired_X = 0
        self.Desired_Y = 0
        self.Desired_Z = 0
        self.Desired_Yaw = 0

        self.Error_Z = 0
        self.Error_X = 0
        self.Error_Y = 0
        self.Error_Yaw = 0

        self.IError_Z = 0
        self.IError_X = 0
        self.IError_Y = 0
        self.IError_Yaw = 0

    def update_desired_position(self, desired_x, desired_y, desired_z, desired_yaw):
        self.Desired_X = desired_x
        self.Desired_Y = desired_y
        self.Desired_Z = desired_z
        self.Desired_Yaw = desired_yaw

    def update_error(self, dp1):

        delayed_error_x = self.Error_X
        delayed_error_y = self.Error_Y
        delayed_error_z = self.Error_Z
        delayed_error_yaw = self.Error_Yaw

        self.Error_X = self.Desired_X - dp1.X_F
        self.Error_Y = self.Desired_Y - dp1.Y_F
        self.Error_Z = self.Desired_Z - dp1.Z_F
        self.Error_Yaw = self.Desired_Yaw - dp1.angleY

        d_error_x = (self.Error_X - delayed_error_x) / self.sample_time
        d_error_y = (self.Error_Y - delayed_error_y) / self.sample_time
        d_error_z = (self.Error_Z - delayed_error_z) / self.sample_time
        d_error_yaw = (self.Error_Yaw - delayed_error_yaw) / self.sample_time

        # Tustin Approximation
        self.IError_X = self.IError_X + ((self.Error_X + delayed_error_x) * self.sample_time / 2)
        self.IError_Y = self.IError_Y + ((self.Error_Y + delayed_error_y) * self.sample_time / 2)
        self.IError_Z = self.IError_Z + ((self.Error_Z + delayed_error_z) * self.sample_time / 2)
        self.IError_Yaw = self.IError_Yaw + ((self.Error_Yaw + delayed_error_yaw) * self.sample_time / 2)

        # PID controller
        U_X = self.k_p_x * self.Error_X + self.k_i_x_temp * self.IError_X + self.k_d_x * d_error_x
        U_Y = self.k_p_y * self.Error_Y + self.k_i_y_temp * self.IError_Y + self.k_d_y * d_error_y
        U_Z = self.k_p_z * self.Error_Z + self.k_i_z_temp * self.IError_Z + self.k_d_z * d_error_z
        U_Yaw = self.k_p_yaw * self.Error_Yaw + self.k_i_yaw_temp * self.IError_Yaw + self.k_d_yaw * d_error_yaw

        return U_X, U_Y, U_Z, U_Yaw

    def integrator_enable(self):
        self.k_i_x_temp = self.k_i_x
        self.k_i_y_temp = self.k_i_y
        self.k_i_z_temp = self.k_i_z
        self.k_i_yaw_temp = self.k_i_yaw

